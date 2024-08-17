from cereal import car
from openpilot.selfdrive.car import DT_CTRL, apply_meas_steer_torque_limits
from openpilot.selfdrive.car.bmw import bmwcan
from openpilot.selfdrive.car.bmw.bmwcan import SteeringModes, CruiseStalk
from openpilot.selfdrive.car.bmw.values import CarControllerParams, CanBus, BmwFlags
from openpilot.selfdrive.car.interfaces import CarControllerBase
from opendbc.can.packer import CANPacker
from openpilot.selfdrive.car.conversions import Conversions as CV

VisualAlert = car.CarControl.HUDControl.VisualAlert

CC_STEP = 1 # cruise single click jump - either km or miles
# Accel limits
ACCEL_HYST_GAP = 0.02  # don't change accel command for small oscilalitons within this value
ACCEL_MAX = 4  # cruise control rapid clicking
ACCEL_SLOW = 3 # cruise control hold up
DECEL_SLOW = -2   # cruise control decrease speed slowly
DECEL_MIN = -6  # cruise control hold down
ACCEL_SCALE = max(ACCEL_MAX, -DECEL_MIN)


class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP):
    super().__init__(dbc_name, CP)
    self.flags = CP.flags


    self.braking = False
    # redundant safety check with the board
    self.last_controls_enabled = False
    self.apply_steer_last = 0
    self.accel_steady = 0.
    self.last_frame_cruise_cmd_sent = 0
    self.last_accel_req = 0
    self.cruise_speed_prev = 0
    self.calcDesiredSpeed = 0
    self.cruise_counter = 0
    self.stock_cruise_counter_last = -1

    self.cruise_bus = CanBus.PT_CAN
    if CP.flags & BmwFlags.DYNAMIC_CRUISE_CONTROL:
      self.cruise_bus = CanBus.F_CAN

    self.packer = CANPacker(dbc_name)

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    # detect check driver pressing as well
    # check whether speed changed in the direction that was commanded - covers OP and driver's commands
    if (self.cruise_speed_prev - CS.out.cruiseState.speed) != 0:
      self.last_accel_req = self.cruise_speed_prev - CS.out.cruiseState.speed

    frames_since_cruise_sent = (self.frame - self.last_frame_cruise_cmd_sent)
    # it should only take one frame for car to update cruise speed. If cruise speed changed after two frames,
    # it was probably driver pressing stalks. In that case update the timestamp too
    if frames_since_cruise_sent > 1.5: # ignore if cruiseState.speed changed shortly after sending command
      self.last_frame_cruise_cmd_sent = self.frame

    time_since_cruise_sent = frames_since_cruise_sent * DT_CTRL

    # *** desired speed model ***
    if abs(actuators.accel) < 0.1:
      self.calcDesiredSpeed = CS.out.vEgo
    self.calcDesiredSpeed = self.calcDesiredSpeed + actuators.accel * DT_CTRL

    # hysteresis
    speed_diff_req = (self.calcDesiredSpeed - CS.out.cruiseState.speed) * (CV.MS_TO_KPH if CS.is_metric else CV.MS_TO_MPH)
    speed_margin_thresh = 0.1
    hysteresis_timeout = 0.2
    # hysteresis, cruiseState.speed changes in steps
    if self.last_accel_req > 0 and actuators.accel > 0.2 and time_since_cruise_sent < hysteresis_timeout:
      speed_diff_err_up = speed_margin_thresh
      speed_diff_err_dn =  -CC_STEP
    elif self.last_accel_req < 0 and actuators.accel < 0.2 and time_since_cruise_sent < hysteresis_timeout:
      speed_diff_err_up =  CC_STEP
      speed_diff_err_dn = -speed_margin_thresh
    else:
      speed_diff_err_up = CC_STEP / 2 + speed_margin_thresh
      speed_diff_err_dn = -CC_STEP / 2

    # *** stalk press rate ***
    if (actuators.accel < 0.2 or actuators.accel > 0.4) and abs(speed_diff_req) > CC_STEP * 1.5:
      # actuators.accel values ^^ inspired by C0F_VERZOEG_POS_FEIN, C0F_VERZOEG_NEG_FEIN from NCSDummy
      cruise_tick = 0.05   # emulate held stalk (keep sending messages at 100Hz) to make bmw brake or accelerate hard
      accel = 2
    else:
      cruise_tick = 0.2 # default rate when not holding stalk
      accel = 1

    # *** cruise control counter handling ***
    # detect stock CruiseControlStalk message counter change - message arrives at only 5Hz when idle
    if self.stock_cruise_counter_last != CS.cruise_counter:
      self.cruise_counter = CS.cruise_counter
    self.stock_cruise_counter_last = CS.cruise_counter

    if CC.cruiseControl.cancel and time_since_cruise_sent > cruise_tick:
      self.cruise_counter = self.cruise_counter + 1
      can_sends.append(bmwcan.create_accel_command(self.packer, CruiseStalk.cancel, self.cruise_bus, self.cruise_counter))
      self.last_frame_cruise_cmd_sent = self.frame
      self.last_accel_req = 0
      print("cancel")
    elif CC.cruiseControl.resume and time_since_cruise_sent > cruise_tick:
      self.cruise_counter = self.cruise_counter + 1
      can_sends.append(bmwcan.create_accel_command(self.packer, CruiseStalk.resume, self.cruise_bus, self.cruise_counter))
      self.last_frame_cruise_cmd_sent = self.frame
      self.last_accel_req = accel
    elif speed_diff_req > speed_diff_err_up and CC.enabled and time_since_cruise_sent > cruise_tick:
      self.cruise_counter = self.cruise_counter + 1
      can_sends.append(bmwcan.create_accel_command(self.packer, CruiseStalk.plus1, self.cruise_bus, self.cruise_counter))
      self.last_frame_cruise_cmd_sent = self.frame
      self.last_accel_req = accel
    elif speed_diff_req < speed_diff_err_dn and CC.enabled and time_since_cruise_sent > cruise_tick and not CS.out.gasPressed:
      self.cruise_counter = self.cruise_counter + 1
      can_sends.append(bmwcan.create_accel_command(self.packer, CruiseStalk.minus1, self.cruise_bus, self.cruise_counter))
      self.last_frame_cruise_cmd_sent = self.frame
      self.last_accel_req = -accel

    self.cruise_speed_prev = CS.out.cruiseState.speed

    if self.flags & BmwFlags.STEPPER_SERVO_CAN:
      # *** apply steering torque ***
      apply_steer = 0
      if CC.enabled and not CS.sportMode:
        new_steer = actuators.steer * CarControllerParams.STEER_MAX
        # explicitly clip torque before sending on CAN
        apply_steer = apply_meas_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorqueEps, CarControllerParams)

        can_sends.append(bmwcan.create_steer_command(self.frame, SteeringModes.TorqueControl, apply_steer))
        # *** control msgs ***
        if (self.frame % 10) == 0: #slow print
          brake_torque = actuators.accel
          frame_number = self.frame
          print(f"Steering req: {actuators.steer}, Brake torque: {brake_torque}, Frame number: {frame_number}")
      elif (not CC.enabled and self.last_controls_enabled):  # cancel on falling edge
        can_sends.append(bmwcan.create_steer_command(self.frame, SteeringModes.Off))

      self.apply_steer_last = apply_steer
      self.last_controls_enabled = CC.enabled

    new_actuators = actuators.as_builder()
    new_actuators.steer = self.apply_steer_last / CarControllerParams.STEER_MAX
    new_actuators.steerOutputCan = self.apply_steer_last

    new_actuators.accel = self.last_accel_req
    new_actuators.speed = self.calcDesiredSpeed

    self.frame += 1
    return new_actuators, can_sends
