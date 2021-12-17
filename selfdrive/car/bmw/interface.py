#!/usr/bin/env python3
from cereal import car
from common.numpy_fast import clip, interp
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.drive_helpers import EventTypes as ET, create_event
from selfdrive.car.bmw.values import CM, BP, AH, CAR, SteerLimitParams, SteerActuatorParams
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, is_ecu_disconnected, gen_empty_fingerprint
from selfdrive.swaglog import cloudlog
from selfdrive.car.interfaces import CarInterfaceBase

ButtonType = car.CarState.ButtonEvent.Type


# certian driver intervention can be distinguished from road disturbance by estimating limits for natural motion due to centering
def detect_stepper_override(steerCmd, steerAct, vEgo, centering_ceoff, SteerFrictionTq):
  # when steering released (or lost steps), what angle will it return to
  # if we are above that angle, we can detect things
  releaseAngle = SteerFrictionTq / (max(vEgo, 1) ** 2 * centering_ceoff)

  override = False
  marginVal = 1
  if abs(steerCmd) > releaseAngle:  # for higher angles we steering will not move outward by itself with stepper on
    if steerCmd > 0:
      override |= steerAct - steerCmd > marginVal  # driver overrode from right to more right
      override |= steerAct < 0  # releaseAngle -3  # driver overrode from right to opposite direction
    else:
      override |= steerAct - steerCmd < -marginVal  # driver overrode from left to more left
      override |= steerAct > 0  # -releaseAngle +3 # driver overrode from left to opposite direction
  # else:
    # override |= abs(steerAct) > releaseAngle + marginVal  # driver overrode to an angle where steering will not go by itself
  return override


class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState)
    
    self.cp_F = self.CS.get_F_can_parser(CP)

    self.enabled = False
    self.gas_pressed_prev3 = False
    self.gas_pressed_prev2 = False
    self.steeringAngle_prev = 0. # it's ok for first sample to be wrong
    self.steeringActuatorEnabled_prev = False

  @staticmethod
  def compute_gb(accel, speed):
    return float(accel) / 3.0

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), vin="", has_relay=False):
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint, False)

    ret.carName = "bmw"
    ret.safetyModel = car.CarParams.SafetyModel.bmw
    ret.steerControlType = car.CarParams.SteerControlType.angle
    ret.steerActuatorDelay = 0.15
    ret.steerLimitTimer = 5


    ret.lateralTuning.init('pid')
    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[1], [0.]]
    ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.01], [0.]]
    ret.lateralTuning.pid.kf = SteerActuatorParams.CENTERING_COEFF
    ret.steerMaxBP = [0.]
    ret.steerMaxV = [5.]

    if candidate in [CAR.E82_DCC, CAR.E82]:
      # stop_and_go = False
      ret.safetyParam = 66  # see conversion factor for STEER_TORQUE_EPS in dbc file
      ret.wheelbase = 2.66
      ret.steerRatio = 16.00
      tire_stiffness_factor = 0.8   # hand-tune
      ret.mass = 3145. * CV.LB_TO_KG + STD_CARGO_KG
    if candidate in [CAR.E90_DCC, CAR.E90]:
      # stop_and_go = False
      ret.safetyParam = 73
      ret.wheelbase = 2.76
      ret.steerRatio = 16.00
      tire_stiffness_factor = 0.8
      ret.mass = 3300. * CV.LB_TO_KG + STD_CARGO_KG  # mean between normal and hybrid

    #bmw uses cruise control to control speed, no PID needed
    ret.openpilotLongitudinalControl = True
    ret.longitudinalTuning.kpBP = [0.]
    ret.longitudinalTuning.kpV = [.1]
    ret.longitudinalTuning.kiBP = [0.]
    ret.longitudinalTuning.kiV = [0.]
    ret.longitudinalTuning.deadzoneBP = [0.]
    ret.longitudinalTuning.deadzoneV = [0.1]

    ret.steerRateCost = 1.
    ret.centerToFront = ret.wheelbase * 0.44

    # min speed to enable ACC. if car can do stop and go, then set enabling speed
    # to a negative value, so it won't matter.
    is_metric = True
    if candidate in [CAR.E82_DCC, CAR.E90_DCC]:  # DCC imperial has higher threshold
      ret.minEnableSpeed = 15. * CV.KPH_TO_MS if is_metric else 20. * CV.MPH_TO_MS #todo merge with carstate is_metric
    elif candidate in [CAR.E82, CAR.E90]:
      ret.minEnableSpeed = 18. * CV.MPH_TO_MS
    # TODO: get actual value, for now starting with reasonable value for
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)

    ret.enableCamera = True  # Enables OpenPilot, if false then passive behaviour
    ret.enableDsu = False

    ret.stoppingControl = False
    ret.startAccel = 0.0
    print("Controler: " + ret.lateralTuning.which())


    return ret

  # returns a car.CarState
  def update(self, c, can_strings):
    # ******************* do can recv *******************
    self.cp.update_strings(can_strings)
    self.cp_F.update_strings(can_strings)

    ret = self.CS.update(self.cp, self.cp_F)
    ret.canValid = self.cp.can_valid and self.cp_F.can_valid

    ret.yawRate = self.VM.yaw_rate(ret.steeringAngle * CV.DEG_TO_RAD, ret.vEgo)
    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False

    buttonEvents = []
    #cruise button events - used to change target speed
    if self.CS.right_blinker_pressed:
      be = car.CarState.ButtonEvent.new_message()
      be.type = ButtonType.rightBlinker
      be.pressed = True
      buttonEvents.append(be)
    if self.CS.left_blinker_pressed:
      be = car.CarState.ButtonEvent.new_message()
      be.type = ButtonType.leftBlinker
      be.pressed = True
      buttonEvents.append(be)
    if self.CS.cruise_plus != self.CS.prev_cruise_plus:
      be = car.CarState.ButtonEvent.new_message()
      print(self.CS.cruise_plus)
      be.type = ButtonType.accelCruise
      be.pressed = self.CS.cruise_plus  #true in rising edge
      buttonEvents.append(be)
    if self.CS.cruise_minus != self.CS.prev_cruise_minus:
      be = car.CarState.ButtonEvent.new_message()
      be.type = ButtonType.decelCruise
      be.pressed = self.CS.cruise_minus  #true in rising edge
      buttonEvents.append(be)
    if self.CS.cruise_resume != self.CS.prev_cruise_resume:
      be = car.CarState.ButtonEvent.new_message()
      be.type = ButtonType.resumeCruise
      be.pressed = self.CS.cruise_resume  #true in rising edge
      buttonEvents.append(be)
    if self.CS.cruise_cancel != self.CS.prev_cruise_cancel:
      be = car.CarState.ButtonEvent.new_message()
      be.type = ButtonType.cancel
      be.pressed = self.CS.cruise_cancel  #true in rising edge
      buttonEvents.append(be)
    if self.CS.otherButtons:
      be = car.CarState.ButtonEvent.new_message()
      be.type = ButtonType.altButton1
      be.pressed = True
      buttonEvents.append(be)

    ret.buttonEvents = buttonEvents

    # events
    events = self.create_common_events(ret, [], gas_resume_speed = 999 * CV.MPH_TO_MS)
    if ret.vEgo < self.CP.minEnableSpeed and self.CP.openpilotLongitudinalControl:
      events.append(create_event('speedTooLow', [ET.NO_ENTRY]))
      if c.actuators.gas > 0.1:
        print("too low speed and actuator.gas > 0.1 NO_ENTRY")
        # some margin on the actuator to not false trigger cancellation while stopping
        events.append(create_event('speedTooLow', [ET.IMMEDIATE_DISABLE]))
      if ret.vEgo < self.CP.minEnableSpeed-1:
        # while in standstill, send a user alert
        events.append(create_event('manualRestart', [ET.WARNING]))
    # disable on pedals rising edge or when brake is pressed and speed isn't zero
    # # filter-out single gas_press drop
    # if (ret.gasPressed and not self.gas_pressed_prev and not self.gas_pressed_prev2 and not self.gas_pressed_prev3) or  \
    #    (ret.brakePressed and (not self.brake_pressed_prev or ret.vEgo > 0.001)):
    #   events.append(create_event('pedalPressed', [ET.NO_ENTRY, ET.USER_DISABLE]))

    if ret.cruiseState.enabled and not self.cruise_enabled_prev:
      events.append(create_event('pcmEnable', [ET.ENABLE]))
    if   self.CS.cruise_resume and not self.CS.prev_cruise_resume and self.cruise_enabled_prev and ret.cruiseState.enabled and self.enabled: #stay in cruise control, but disable OpenPilot
      events.append(create_event('buttonCancel', [ET.USER_DISABLE]))
    elif self.CS.cruise_resume and not self.CS.prev_cruise_resume and self.cruise_enabled_prev and ret.cruiseState.enabled and not self.enabled:  #when in cruise control, press resume to resume OpenPilot
      events.append(create_event('buttonEnable', [ET.ENABLE]))
    if self.CS.cruise_cancel:
      events.append(create_event('buttonCancel', [ET.USER_DISABLE]))
    # if self.CS.gas_kickdown:
    #   events.append(create_event('pedalPressed', [ET.NO_ENTRY, ET.USER_DISABLE]))

    steeringActuatorEnabled = self.enabled and not self.CS.sportMode
    # wait for steering actuator to be enabled for two samples to get stepper delta calculated correctly
    # if steeringActuatorEnabled and self.steeringActuatorEnabled_prev and detect_stepper_override(steerCmdLimited_prev, ret.steeringAngle, ret.vEgo, self.CP.lateralTuning.pid.kf, SteerActuatorParams.STEER_TORQUE_OFFSET):
    #    events.append(create_event('steerUnavailable', [ET.IMMEDIATE_DISABLE]))  #TODO clean up this event - detect driver overriding controls
    self.steeringActuatorEnabled_prev = steeringActuatorEnabled

    # update previous brake/gas pressed
    self.steeringAngle_prev = ret.steeringAngle
    self.gas_pressed_prev3 = self.gas_pressed_prev2
    self.gas_pressed_prev2 = self.gas_pressed_prev
    self.gas_pressed_prev = ret.gasPressed
    self.brake_pressed_prev = ret.brakePressed
    self.cruise_enabled_prev = ret.cruiseState.enabled

    ret.events = events
    self.CS.out = ret.as_reader()
    return self.CS.out

  # pass in a car.CarControl
  # to be called @ 100hz
  def apply(self, c):

    self.enabled = c.enabled
    can_sends = self.CC.update(c, self.CS, self.frame)
                               # c.actuators, c.cruiseControl,
                               # c.hudControl.visualAlert, c.hudControl.leftLaneVisible,
                               # c.hudControl.rightLaneVisible, c.hudControl.leadVisible,
                               # c.hudControl.leftLaneDepart, c.hudControl.rightLaneDepart)

    self.frame += 1
    return can_sends
