#!/usr/bin/env python3
from cereal import car
from common.numpy_fast import clip, interp
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.drive_helpers import EventTypes as ET, create_event
from selfdrive.car.bmw.values import CM, BP, AH, CAR, FINGERPRINTS
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, is_ecu_disconnected, gen_empty_fingerprint
from selfdrive.swaglog import cloudlog
from selfdrive.car.interfaces import CarInterfaceBase

class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState)
    self.gas_pressed_prev3 = False
    self.gas_pressed_prev2 = False
    self.gas_pressed_prev1 = False


  @staticmethod
  def compute_gb(accel, speed):
    return float(accel) / 3.0

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), vin="", has_relay=False):
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint, has_relay)

    ret.carName = "bmw"

    ret.safetyModel = car.CarParams.SafetyModel.bmw

    ret.steerActuatorDelay = 0.12  # Default delay, Prius has larger delay
    ret.steerLimitTimer = 0.4

    if True:
      ret.lateralTuning.init('pid')
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]

    if candidate in [CAR.E82_DCC, CAR.E82]:
      # stop_and_go = False
      ret.safetyParam = 66  # see conversion factor for STEER_TORQUE_EPS in dbc file
      ret.wheelbase = 2.66
      ret.steerRatio = 16.00   # unknown end-to-end spec
      tire_stiffness_factor = 0.6371   # hand-tune
      ret.mass = 3145. * CV.LB_TO_KG + STD_CARGO_KG
    if candidate in [CAR.E90_DCC, CAR.E90]:
      # stop_and_go = False
      ret.safetyParam = 73
      ret.wheelbase = 2.90
      ret.steerRatio = 16.00  # unknown end-to-end spec
      tire_stiffness_factor = 0.5533
      ret.mass = 3300. * CV.LB_TO_KG + STD_CARGO_KG  # mean between normal and hybrid


    ret.lateralTuning.pid.kf = 0.00006   # full torque for 10 deg at 80mph means 0.00007818594

    ret.lateralTuning.init('indi')
    ret.lateralTuning.indi.innerLoopGain = 4.0
    ret.lateralTuning.indi.outerLoopGain = 3.0
    ret.lateralTuning.indi.timeConstant = 1.0
    ret.lateralTuning.indi.actuatorEffectiveness = 1.0

    #bmw uses cruise control to control speed, no PID needed
    ret.openpilotLongitudinalControl = True
    ret.longitudinalTuning.kpBP = [0.]
    ret.longitudinalTuning.kpV = [.1]
    ret.longitudinalTuning.kiBP = [0.]
    ret.longitudinalTuning.kiV = [0.]
    ret.longitudinalTuning.deadzoneBP = [0.]
    ret.longitudinalTuning.deadzoneV = [1.]

    ret.steerRateCost = 1.
    ret.centerToFront = ret.wheelbase * 0.44

    # min speed to enable ACC. if car can do stop and go, then set enabling speed
    # to a negative value, so it won't matter.
    if candidate in [CAR.E82_DCC, CAR.E90_DCC]:  # DCC has higher threshold
      ret.minEnableSpeed = (20.-1.)* CV.MPH_TO_MS
    elif candidate in [CAR.E82, CAR.E90]:
      ret.minEnableSpeed = (18.-1.) * CV.MPH_TO_MS
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


    return ret

  # returns a car.CarState
  def update(self, c, can_strings):
    # ******************* do can recv *******************
    self.cp.update_strings(can_strings)
    self.cp_cam.update_strings(can_strings)

    ret = self.CS.update(self.cp, self.cp_cam)

    ret.canValid = self.cp.can_valid and self.cp_cam.can_valid

    ret.yawRate = self.VM.yaw_rate(ret.steeringAngle * CV.DEG_TO_RAD, ret.vEgo)
    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False


    # events
    events = self.create_common_events(ret)
    if self.CS.low_speed_lockout and self.CP.openpilotLongitudinalControl:
      events.append(create_event('lowSpeedLockout', [ET.NO_ENTRY, ET.PERMANENT]))
    if ret.vEgo < self.CP.minEnableSpeed and self.CP.openpilotLongitudinalControl:
      events.append(create_event('speedTooLow', [ET.NO_ENTRY]))
      if c.actuators.gas > 0.1:
        print("too low speed and actuator.gas > 0.1 NO_ENTRY")
        # some margin on the actuator to not false trigger cancellation while stopping
        events.append(create_event('speedTooLow', [ET.IMMEDIATE_DISABLE]))
      if ret.vEgo < 0.001:
        # while in standstill, send a user alert
        events.append(create_event('manualRestart', [ET.WARNING]))
    # disable on pedals rising edge or when brake is pressed and speed isn't zero
    # # filter-out single gas_press drop
    # if (ret.gasPressed and not self.gas_pressed_prev1 and not self.gas_pressed_prev2 and not self.gas_pressed_prev3) or  \
    #    (ret.brakePressed and (not self.brake_pressed_prev or ret.vEgo > 0.001)):
    #   events.append(create_event('pedalPressed', [ET.NO_ENTRY, ET.USER_DISABLE]))

    # if ret.gasPressed: # or self.gas_pressed_prev1 or self.gas_pressed_prev2 or self.gas_pressed_prev3:
    #   events.append(create_event('pedalPressed', [ET.PRE_ENABLE]))
    if ret.cruiseState.enabled and not self.cruise_enabled_prev:
      events.append(create_event('pcmEnable', [ET.ENABLE]))
    if   self.CS.cruise_resume and not self.CS.prev_cruise_resume and self.cruise_enabled_prev and ret.cruiseState.enabled and self.enabled: #stay in cruise control, but disable OpenPilot
      events.append(create_event('buttonCancel', [ET.USER_DISABLE]))
    elif self.CS.cruise_resume and not self.CS.prev_cruise_resume and self.cruise_enabled_prev and ret.cruiseState.enabled and not self.enabled:  #when in cruise control, press resume to resume OpenPilot
      events.append(create_event('buttonEnable', [ET.ENABLE]))
    if self.CS.cruise_cancel:
      events.append(create_event('buttonCancel', [ET.USER_DISABLE]))
    ret.events = events

    # update previous brake/gas pressed
    self.gas_pressed_prev3 = self.gas_pressed_prev2
    self.gas_pressed_prev2 = self.gas_pressed_prev1
    self.gas_pressed_prev1 = ret.gasPressed
    self.brake_pressed_prev = ret.brakePressed
    self.cruise_enabled_prev = ret.cruiseState.enabled

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
