from cereal import car
from openpilot.selfdrive.car import DT_CTRL, apply_dist_to_meas_limits, apply_hysteresis
from openpilot.selfdrive.car.bmw import bmwcan
from openpilot.selfdrive.car.bmw.bmwcan import SteeringModes, CruiseStalk
from openpilot.selfdrive.car.bmw.values import CarControllerParams, CanBus, BmwFlags, CruiseSettings
from openpilot.selfdrive.car.interfaces import CarControllerBase
from opendbc.can.packer import CANPacker
from openpilot.selfdrive.car.conversions import Conversions as CV

VisualAlert = car.CarControl.HUDControl.VisualAlert

CC_STEP = 1 # cruise single click jump - always 1 - interpreted as km or miles depending on DSC or DME set units

# Accel limits
ACCEL_HYST_GAP = CC_STEP * 0.9  # shall be between half or full crusie step
ACCEL_MAX = 4  # cruise control rapid clicking
ACCEL_SLOW = 3 # cruise control hold up
DECEL_SLOW = -2   # cruise control decrease speed slowly
DECEL_MIN = -6  # cruise control hold down
ACCEL_SCALE = max(ACCEL_MAX, -DECEL_MIN)

STOCK_CRUISE_STALK_TICK = 0.2 # sample rate of stock cruise stalk messages when not pressed
STOCK_CRUISE_STALK_HOLD_TICK = 0.05 # sample rate of stock cruise stalk messages when pressed


class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP):
    super().__init__(dbc_name, CP)
    self.flags = CP.flags
    self.minCruiseSpeed = CP.minEnableSpeed

    self.CC_cancel = False  # local cruise control cancel
    self.CC_enabled_prev = False
    # redundant safety check with the board
    self.apply_steer_last = 0
    self.accel_steady = 0.
    self.last_cruise_cmd_timestamp = 0
    self.last_cruise_speed_delta_req = 0
    self.cruise_speed_with_hyst = 0
    self.actuators_accel_last = 0
    self.calcDesiredSpeed = 0
    self.tx_cruise_stalk_counter = 0
    self.rx_cruise_stalk_counter_last = -1
    self.last_user_steer_cancel = True

    self.cruise_bus = CanBus.PT_CAN
    if CP.flags & BmwFlags.DYNAMIC_CRUISE_CONTROL:
      self.cruise_bus = CanBus.F_CAN


    self.packer = CANPacker(dbc_name)

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    self.CC_units = (CV.MS_TO_KPH if CS.is_metric else CV.MS_TO_MPH)

    # detect acceleration sign change
    accel_zero_cross = actuators.accel * self.actuators_accel_last < 0
    self.actuators_accel_last = actuators.accel

    # *** hysteresis - trend is your friend ***
    # we want to request +/-1 when speed diff is bigger than 0.5
    # cruiseState.speed always changes by CC_STEP, which then would cause oscillations
    # a minimum hysteresis of CC_STEP * 0.5 is required to avoid this
    # a larger hysteresis makes next request to be sent quicker if the speed change continues in the same direction
    self.cruise_speed_with_hyst = apply_hysteresis(CS.out.cruiseState.speed, self.cruise_speed_with_hyst, ACCEL_HYST_GAP / self.CC_units)
    if not CS.out.cruiseState.enabled:
      self.cruise_speed_with_hyst = CS.out.vEgo
    if accel_zero_cross:
      self.cruise_speed_with_hyst = CS.out.cruiseState.speed

    # *** desired speed model ***
    if accel_zero_cross:
      self.calcDesiredSpeed = CS.out.vEgo
    self.calcDesiredSpeed = self.calcDesiredSpeed + actuators.accel * DT_CTRL
    speed_diff_req = (self.calcDesiredSpeed - self.cruise_speed_with_hyst) * self.CC_units
    # *** stalk press rate ***
    if (actuators.accel < -0.2 or actuators.accel > 0.2) and abs(speed_diff_req) > CC_STEP:
      # actuators.accel values ^^ inspired by C0F_VERZOEG_POS_FEIN, C0F_VERZOEG_NEG_FEIN from NCSDummy
      cruise_tick = 0.1   # emulate held stalk (keep sending messages at 100Hz) to make bmw brake or accelerate hard
    else:
      cruise_tick = STOCK_CRUISE_STALK_TICK # default rate when not holding stalk

    # *** cruise control counter handling ***
    # detect incoming CruiseControlStalk message by observing counter change (message arrives at only 5Hz when nothing pressed)
    if CS.cruise_stalk_counter != self.rx_cruise_stalk_counter_last:
      self.tx_cruise_stalk_counter = CS.cruise_stalk_counter + 1
      # stock message was sent some time in between control samples:
      self.last_cruise_cmd_timestamp = now_nanos - DT_CTRL / 2 * 1e9 # assume half of DT_CTRL, #todo can be replaced with precise ts_nanos from can parser
    self.rx_cruise_stalk_counter_last = CS.cruise_stalk_counter

    cruise_stalk_human_pressing = CS.cruise_stalk_plus \
                               or CS.cruise_stalk_minus \
                               or CS.cruise_stalk_plus5 \
                               or CS.cruise_stalk_minus5 \
                               or CS.cruise_stalk_resume \
                               or CS.cruise_stalk_cancel

    time_since_cruise_sent =  (now_nanos - self.last_cruise_cmd_timestamp) / 1e9


    # *** cruise control cancel signal ***
    # CC.cruiseControl.cancel can't be used because it is always false because pcmCruise = False because we need OP speed tracker
    # CC.enabled appears after cruiseState.enabled, so we need to check rising edge to prevent instantaneous cancel after cruise is enabled
    # This is because CC.enabled comes from controld and CS.out.cruiseState.enabled is from card threads
    if not CC.enabled and self.CC_enabled_prev:
      self.CC_cancel = True
    # if we need to go below cruise speed, request cancel and coast while steering enabled
    if CS.out.cruiseState.speedCluster - self.minCruiseSpeed < 0.1 and actuators.accel < 0.1 \
      and CS.out.vEgoCluster - self.minCruiseSpeed < 0.5 and CS.out.vEgo - self.calcDesiredSpeed > 1:
      self.CC_cancel = True
    # keep requesting cancel until the cruise is disabled
    if not CS.out.cruiseState.enabled:
      self.CC_cancel = False

    if not cruise_stalk_human_pressing:
      if self.CC_cancel and CS.out.cruiseState.enabled and time_since_cruise_sent > cruise_tick:
        self.tx_cruise_stalk_counter = self.tx_cruise_stalk_counter + 1
        can_sends.append(bmwcan.create_accel_command(self.packer, CruiseStalk.cancel, self.cruise_bus, self.tx_cruise_stalk_counter))
        self.last_cruise_cmd_timestamp = now_nanos
        self.last_cruise_speed_delta_req = 0
        print("cancel")
      elif CC.enabled and speed_diff_req > CC_STEP/2 and CS.out.cruiseState.enabled and time_since_cruise_sent > cruise_tick:
        self.tx_cruise_stalk_counter = self.tx_cruise_stalk_counter + 1
        can_sends.append(bmwcan.create_accel_command(self.packer, CruiseStalk.plus1, self.cruise_bus, self.tx_cruise_stalk_counter))
        self.last_cruise_cmd_timestamp = now_nanos
        self.last_cruise_speed_delta_req = +CC_STEP
      elif CC.enabled and speed_diff_req < -CC_STEP/2 and CS.out.cruiseState.enabled and time_since_cruise_sent > cruise_tick and not CS.out.gasPressed:
        self.tx_cruise_stalk_counter = self.tx_cruise_stalk_counter + 1
        can_sends.append(bmwcan.create_accel_command(self.packer, CruiseStalk.minus1, self.cruise_bus, self.tx_cruise_stalk_counter))
        self.last_cruise_cmd_timestamp = now_nanos
        self.last_cruise_speed_delta_req = -CC_STEP



    if self.flags & BmwFlags.STEPPER_SERVO_CAN:
      steer_error =  not CC.latActive and CC.enabled
      if not steer_error and not CS.dtc_mode: # stop steer CAN tx when DTC is On or if steering is unavailable (unless user cancels)
        # *** apply steering torque ***
        if CC.enabled:
          new_steer = actuators.steer * CarControllerParams.STEER_MAX
          # explicitly clip torque before sending on CAN
          apply_steer = apply_dist_to_meas_limits(new_steer, self.apply_steer_last, CS.out.steeringTorqueEps,
                                             CarControllerParams.STEER_DELTA_UP, CarControllerParams.STEER_DELTA_DOWN,
                                             CarControllerParams.STEER_ERROR_MAX, CarControllerParams.STEER_MAX)
          can_sends.append(bmwcan.create_steer_command(self.frame, SteeringModes.TorqueControl, apply_steer))
          # *** control msgs ***
          if (self.frame % 10) == 0: #slow print
            brake_torque = actuators.accel
            frame_number = self.frame
            print(f"Steering req: {actuators.steer}, Brake torque: {brake_torque}, Frame number: {frame_number}")
        else:
          apply_steer = 0
          can_sends.append(bmwcan.create_steer_command(self.frame, SteeringModes.Off))
        self.apply_steer_last = apply_steer

    new_actuators = actuators.as_builder()
    new_actuators.steer = self.apply_steer_last / CarControllerParams.STEER_MAX
    new_actuators.steerOutputCan = self.apply_steer_last

    new_actuators.speed = self.calcDesiredSpeed
    new_actuators.accel = speed_diff_req

    self.frame += 1
    return new_actuators, can_sends
