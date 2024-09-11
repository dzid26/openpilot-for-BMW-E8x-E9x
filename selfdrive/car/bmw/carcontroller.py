from cereal import car
from openpilot.selfdrive.car import DT_CTRL, apply_dist_to_meas_limits, apply_hysteresis
from openpilot.selfdrive.car.bmw import bmwcan
from openpilot.selfdrive.car.bmw.bmwcan import SteeringModes, CruiseStalk
from openpilot.selfdrive.car.bmw.values import CarControllerParams, CanBus, BmwFlags
from openpilot.selfdrive.car.interfaces import CarControllerBase
from opendbc.can.packer import CANPacker
from openpilot.selfdrive.car.conversions import Conversions as CV

VisualAlert = car.CarControl.HUDControl.VisualAlert

# DO NOT CHANGE: Cruise control step size
CC_STEP = 1 # cruise single click jump - always 1 - interpreted as km or miles depending on DSC or DME set units
ACCEL_HYST_GAP = 0.9 * CC_STEP
CRUISE_STALK_IDLE_TICK_STOCK = 0.2 # stock cruise stalk CAN frequency when stalk is not pressed is 5Hz
CRUISE_STALK_HOLD_TICK_STOCK = 0.01 # stock cruise stalk CAN frequency when stalk is pressed is 20Hz

CRUISE_STALK_SINGLE_TICK = CRUISE_STALK_IDLE_TICK_STOCK # we will send also at 5Hz in between stock messages to emulate single presses
CRUISE_STALK_HOLD_TICK = CRUISE_STALK_HOLD_TICK_STOCK # emulate held stalk, use only fully divisible values, i.e. 0.05, 0.02, 0.01

ACCEL_HYST_GAP = CC_STEP * 0.8  # between 0 and CC_STEP

class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP):
    super().__init__(dbc_name, CP)
    self.flags = CP.flags
    self.min_cruise_speed = CP.minEnableSpeed
    self.cruise_units = None

    self.cruise_cancel = False  # local cruise control cancel
    self.cruise_enabled_prev = False
    # redundant safety check with the board
    self.apply_steer_last = 0
    self.last_cruise_rx_timestamp = 0 # stock cruise buttons
    self.last_cruise_tx_timestamp = 0 # openpilot commands
    self.tx_cruise_stalk_counter_last = 0
    self.rx_cruise_stalk_counter_last = -1
    self.cruise_speed_with_hyst = 0
    self.actuators_accel_last = 0
    self.calc_desired_speed = 0
    self.hold_cruise_accel = False # holds to keep accelerating, cruiseState.speed will be ~ 3kph above vEgo
    self.hold_cruise_accel_5 = False

    self.cruise_bus = CanBus.PT_CAN
    if CP.flags & BmwFlags.DYNAMIC_CRUISE_CONTROL:
      self.cruise_bus = CanBus.F_CAN


    self.packer = CANPacker(dbc_name)


  def update(self, CC, CS, now_nanos):

    actuators = CC.actuators
    can_sends = []

    self.cruise_units = (CV.MS_TO_KPH if CS.is_metric else CV.MS_TO_MPH)

    # detect acceleration sign change
    accel_zero_cross = actuators.accel * self.actuators_accel_last < 0
    self.actuators_accel_last = actuators.accel

    # *** hysteresis - trend is your friend ***
    # avoids cruise speed toggling and biases next request toward the direction of the previous one
    self.cruise_speed_with_hyst = apply_hysteresis(CS.out.cruiseState.speed, self.cruise_speed_with_hyst, ACCEL_HYST_GAP / self.cruise_units)
    if not CS.out.cruiseState.enabled:
      self.cruise_speed_with_hyst = CS.out.vEgo
    if accel_zero_cross:
      self.cruise_speed_with_hyst = CS.out.cruiseState.speed

    # *** desired speed model ***
    if accel_zero_cross or not CC.enabled or CS.out.gasPressed:
      self.calc_desired_speed = CS.out.vEgo
    self.calc_desired_speed = self.calc_desired_speed + actuators.accel * DT_CTRL
    speed_diff_req = (self.calc_desired_speed - self.cruise_speed_with_hyst) * self.cruise_units

    # detect incoming CruiseControlStalk message by observing counter change (message arrives at only 5Hz when nothing pressed)
    if CS.cruise_stalk_counter != self.rx_cruise_stalk_counter_last:
      self.tx_cruise_stalk_counter_last = CS.cruise_stalk_counter
      # stock message was sent some time in between control samples:
      self.last_cruise_rx_timestamp = now_nanos #todo can be replaced with precise ts_nanos from can parser
    self.rx_cruise_stalk_counter_last = CS.cruise_stalk_counter

    cruise_stalk_human_pressing = CS.cruise_stalk_plus \
                               or CS.cruise_stalk_minus \
                               or CS.cruise_stalk_plus5 \
                               or CS.cruise_stalk_minus5 \
                               or CS.cruise_stalk_resume \
                               or CS.cruise_stalk_cancel


    # *** cruise control cancel signal ***
    # CC.cruiseControl.cancel can't be used because it is always false because pcmCruise = False because we need OP speed tracker
    # CC.enabled appears after cruiseState.enabled, so we need to check rising edge to prevent instantaneous cancel after cruise is enabled
    # This is because CC.enabled comes from controld and CS.out.cruiseState.enabled is from card threads
    if not CC.enabled and self.cruise_enabled_prev:
      self.cruise_cancel = True
    # if we need to go below cruise speed, request cancel and coast while steering enabled
    if CS.out.cruiseState.speedCluster - self.min_cruise_speed < 0.1 and actuators.accel < 0.1 \
      and CS.out.vEgoCluster - self.min_cruise_speed < 0.5 and CS.out.vEgo - self.calc_desired_speed > 1:
      self.cruise_cancel = True
    # keep requesting cancel until the cruise is disabled
    if not CS.out.cruiseState.enabled:
      self.cruise_cancel = False

    # *** send cruise control stalk message at different rates and manage counters ***
    def cruise_cmd(cmd, hold=False):
      time_since_cruise_sent =      (now_nanos - self.last_cruise_tx_timestamp) / 1e9 + DT_CTRL / 10 # add half task sample time to account for latency
      time_since_cruise_received =  (now_nanos - self.last_cruise_rx_timestamp) / 1e9 + DT_CTRL / 10 # add half task sample time to account for latency
      # send single cmd with an effective rate slower than held stalk rate
      if not hold:
        send = time_since_cruise_sent > CRUISE_STALK_SINGLE_TICK \
          and time_since_cruise_received > CRUISE_STALK_HOLD_TICK_STOCK/2 - DT_CTRL \
          and time_since_cruise_received < CRUISE_STALK_IDLE_TICK_STOCK/2 + DT_CTRL
      else:
        # use faster rate to emulate held stalk. Time first message such that subsequent one will nullify stock message:
        send = hold and time_since_cruise_sent > CRUISE_STALK_HOLD_TICK
      if send:
        tx_cruise_stalk_counter = self.tx_cruise_stalk_counter_last + 1
        # avoid counter clash with a potential upcoming message from stock cruise
        if tx_cruise_stalk_counter == CS.cruise_stalk_counter + 1:
          # avoid clashing with upcoming stock message
          # sometimes upcoming stock message is overshadowed by us, so also avoid clashing with one after that
          tx_cruise_stalk_counter = tx_cruise_stalk_counter + 2
        tx_cruise_stalk_counter = tx_cruise_stalk_counter % 0xF
        can_sends.append(bmwcan.create_accel_command(self.packer, cmd, self.cruise_bus, tx_cruise_stalk_counter))
        self.tx_cruise_stalk_counter_last = tx_cruise_stalk_counter
        self.last_cruise_tx_timestamp = now_nanos

    if not cruise_stalk_human_pressing:
      if self.cruise_cancel and CS.out.cruiseState.enabled:
        cruise_cmd(CruiseStalk.cancel)
        print("cancel")
      elif CC.enabled and CS.out.cruiseState.enabled:
        if actuators.accel > 1.0:
          cruise_cmd(CruiseStalk.plus5, hold=True) # produces up to 1.2 m/s2
        elif actuators.accel > 0.3:
          cruise_cmd(CruiseStalk.plus1, hold=True) # produces up to 0.8 m/s2
        elif speed_diff_req > CC_STEP/2 and actuators.accel >=0.0: # 0.0 if gasPressed
          cruise_cmd(CruiseStalk.plus1)
      elif CC.enabled and CS.out.cruiseState.enabled and not CS.out.gasPressed:
        if actuators.accel < -1.0
          cruise_cmd(CruiseStalk.minus5, hold=True) # produces down to -1.4 m/s2
        elif actuators.accel < -0.3:
          cruise_cmd(CruiseStalk.minus1, hold=True) # produces down to -0.8 m/s2
        elif speed_diff_req < -CC_STEP/2 and actuators.accel < 0.0:
          cruise_cmd(CruiseStalk.minus1)



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

    self.cruise_enabled_prev = CC.enabled

    new_actuators = actuators.as_builder()
    new_actuators.steer = self.apply_steer_last / CarControllerParams.STEER_MAX
    new_actuators.steerOutputCan = self.apply_steer_last

    new_actuators.speed = self.calc_desired_speed
    new_actuators.accel = speed_diff_req

    self.frame += 1
    return new_actuators, can_sends
