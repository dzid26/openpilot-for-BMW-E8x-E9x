#!/usr/bin/env python3
from cereal import car
from common.numpy_fast import clip, interp
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.drive_helpers import EventTypes as ET, create_event
from selfdrive.controls.lib.vehicle_model import VehicleModel
from selfdrive.car.bmw.carstate import CarState, get_PT_can_parser, get_F_can_parser
from selfdrive.car.bmw.values import CM, BP, AH, CAR, FINGERPRINTS
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, is_ecu_disconnected, gen_empty_fingerprint
from selfdrive.swaglog import cloudlog
from selfdrive.car.interfaces import CarInterfaceBase
from selfdrive.controls.lib.planner import _A_CRUISE_MAX_V

A_ACC_MAX = max(_A_CRUISE_MAX_V)

ButtonType = car.CarState.ButtonEvent.Type
GearShifter = car.CarState.GearShifter

class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController):
    self.CP = CP
    self.VM = VehicleModel(CP)

    self.frame = 0
    self.gas_pressed_prev1 = False
    self.gas_pressed_prev2 = False
    self.gas_pressed_prev3 = False
    self.brake_pressed_prev = False
    self.cruise_enabled_prev = False
    self.enabled = False
    # *** init the major players ***
    self.CS = CarState(CP)

    self.cp_PT = get_PT_can_parser(CP)
    self.cp_F = get_F_can_parser(CP)

    self.CC = None
    if CarController is not None:
      self.CC = CarController(self.cp_PT.dbc_name, CP.carFingerprint, CP.enableCamera, CP.enableDsu, CP.enableApgs)

  @staticmethod
  def compute_gb(accel, speed):
    return float(accel) / 3.0

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), vin="", has_relay=False):

    ret = car.CarParams.new_message()

    ret.carName = "bmw"
    ret.carFingerprint = candidate
    ret.carVin = vin
    ret.isPandaBlack = has_relay

    ret.safetyModel = car.CarParams.SafetyModel.bmw

    ret.enableCruise = True

    ret.steerActuatorDelay = 0.12  # Default delay, Prius has larger delay
    ret.steerLimitTimer = 0.4

    if True:
      ret.lateralTuning.init('pid')
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]

    if candidate in [CAR.E82_DCC, CAR.E82]:
      stop_and_go = False
      ret.safetyParam = 66  # see conversion factor for STEER_TORQUE_EPS in dbc file
      ret.wheelbase = 2.66
      ret.steerRatio = 16.00   # unknown end-to-end spec
      tire_stiffness_factor = 0.6371   # hand-tune
      ret.mass = 3145. * CV.LB_TO_KG + STD_CARGO_KG
    if candidate in [CAR.E90_DCC, CAR.E90]:
      stop_and_go = False
      ret.safetyParam = 73
      ret.wheelbase = 2.90
      ret.steerRatio = 16.00  # unknown end-to-end spec
      tire_stiffness_factor = 0.5533
      ret.mass = 3300. * CV.LB_TO_KG + STD_CARGO_KG  # mean between normal and hybrid


    # TODO: Determine if this is better than INDI
    # ret.lateralTuning.init('lqr')
    # ret.lateralTuning.lqr.scale = 1500.0
    # ret.lateralTuning.lqr.ki = 0.01
    # ret.lateralTuning.lqr.a = [0., 1., -0.22619643, 1.21822268]
    # ret.lateralTuning.lqr.b = [-1.92006585e-04, 3.95603032e-05]
    # ret.lateralTuning.lqr.c = [1., 0.]
    # ret.lateralTuning.lqr.k = [-110.73572306, 451.22718255]
    # ret.lateralTuning.lqr.l = [0.03233671, 0.03185757]
    # ret.lateralTuning.lqr.dcGain = 0.002237852961363602

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

    # no rear steering, at least on the listed cars above
    ret.steerRatioRear = 0.
    ret.steerControlType = car.CarParams.SteerControlType.torque

    # steer, gas, brake limitations VS speed
    ret.steerMaxBP = [16. * CV.KPH_TO_MS, 45. * CV.KPH_TO_MS]  # breakpoints at 1 and 40 kph
    ret.steerMaxV = [1., 1.]  # 2/3rd torque allowed above 45 kph
    ret.brakeMaxBP = [0.]
    ret.gasMaxBP = [0.]
    ret.brakeMaxV = [1.]
    ret.gasMaxV = [0.5]

    ret.enableCamera = True  # Enables OpenPilot, if false then passive behaviour
    ret.enableDsu = False
    ret.enableApgs = False #not check_ecu_msgs(fingerprint, ECU.APGS)

    ret.steerLimitAlert = False
    ret.stoppingControl = False
    ret.startAccel = 0.0


    return ret

  # returns a car.CarState
  def update(self, c, can_strings):
    # ******************* do can recv *******************
    self.cp_PT.update_strings(can_strings)
    self.cp_F.update_strings(can_strings)

    self.CS.update(self.cp_PT, self.cp_F)

    # create message
    ret = car.CarState.new_message()

    ret.canValid = self.cp_PT.can_valid and self.cp_F.can_valid

    # speeds
    ret.vEgo = self.CS.v_ego
    ret.vEgoRaw = self.CS.v_ego_raw
    ret.aEgo = self.CS.a_ego
    ret.yawRate = self.VM.yaw_rate(self.CS.angle_steers * CV.DEG_TO_RAD, self.CS.v_ego)
    ret.standstill = self.CS.standstill
    # ret.wheelSpeeds.fl = self.CS.v_wheel_fl
    # ret.wheelSpeeds.fr = self.CS.v_wheel_fr
    # ret.wheelSpeeds.rl = self.CS.v_wheel_rl
    # ret.wheelSpeeds.rr = self.CS.v_wheel_rr

    # gear shifter
    ret.gearShifter = self.CS.gear_shifter

    # gas pedal
    ret.gas = self.CS.car_gas
    ret.gasPressed = self.CS.gas_pressed
    
    
    
    # brake pedal
    ret.brake = self.CS.user_brake
    ret.brakePressed = self.CS.brake_pressed
    ret.brakeLights = self.CS.brake_lights
    #
    # steering wheel
    ret.steeringAngle = self.CS.angle_steers
    ret.steeringRate = self.CS.angle_steers_rate

    # ret.steeringTorque = self.CS.steer_torque_driver
    # ret.steeringPressed = self.CS.steer_override
    #
    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False

    # cruise state
    ret.cruiseState.enabled = self.CS.pcm_acc_active
    ret.cruiseState.speed = self.CS.v_cruise_pcm * CV.MPH_TO_MS
    ret.cruiseState.available = bool(self.CS.main_on)
    ret.cruiseState.speedOffset = 0
    ret.cruiseState.standstill = False

    buttonEvents = []
    if self.CS.left_blinker_on != self.CS.prev_left_blinker_on:
     be = car.CarState.ButtonEvent.new_message()
     be.type = ButtonType.leftBlinker
     be.pressed = self.CS.left_blinker_on != 0
     buttonEvents.append(be)

    if self.CS.right_blinker_on != self.CS.prev_right_blinker_on:
      be = car.CarState.ButtonEvent.new_message()
      be.type = ButtonType.rightBlinker
      be.pressed = self.CS.right_blinker_on != 0
      buttonEvents.append(be)
    if self.CS.otherButons:
      be = car.CarState.ButtonEvent.new_message()
      be.type = ButtonType.altButton1
      be.pressed = self.CS.otherButons != 0
      buttonEvents.append(be)

    #cruise button events - used to change target speed
    if self.CS.cruise_plus != self.CS.prev_cruise_plus:
      be = car.CarState.ButtonEvent.new_message()
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



    ret.buttonEvents = buttonEvents
    ret.leftBlinker = bool(self.CS.left_blinker_on)
    ret.rightBlinker = bool(self.CS.right_blinker_on)

    #ret.doorOpen = not self.CS.door_all_closed
    #ret.seatbeltUnlatched = not self.CS.seatbelt

    #ret.genericToggle = self.CS.generic_toggle

    # events
    events = []

    if not ret.gearShifter == GearShifter.drive and self.CP.openpilotLongitudinalControl:
      events.append(create_event('wrongGear', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
    # if ret.doorOpen:
    #   events.append(create_event('doorOpen', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
    # if ret.seatbeltUnlatched:
    #   events.append(create_event('seatbeltNotLatched', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
    #if self.CS.esp_disabled and self.CP.openpilotLongitudinalControl:
    #   events.append(create_event('espDisabled', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
    #if not self.CS.main_on and self.CP.openpilotLongitudinalControl:
    #   events.append(create_event('wrongCarMode', [ET.NO_ENTRY, ET.USER_DISABLE]))
    #if ret.gearShifter == GearShifter.reverse and self.CP.openpilotLongitudinalControl:
    #   events.append(create_event('reverseGear', [ET.NO_ENTRY, ET.IMMEDIATE_DISABLE]))
    # if self.CS.steer_error:
    #   events.append(create_event('steerTempUnavailable', [ET.NO_ENTRY, ET.WARNING]))
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
    # filter-out single gas_press drop
    if (ret.gasPressed and not self.gas_pressed_prev1 and not self.gas_pressed_prev2 and not self.gas_pressed_prev3) or  \
       (ret.brakePressed and (not self.brake_pressed_prev or ret.vEgo > 0.001)):
      events.append(create_event('pedalPressed', [ET.NO_ENTRY, ET.USER_DISABLE]))

    if ret.gasPressed: # or self.gas_pressed_prev1 or self.gas_pressed_prev2 or self.gas_pressed_prev3:
      events.append(create_event('pedalPressed', [ET.PRE_ENABLE]))
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

    # cast to reader so it can't be modified
    return ret.as_reader()

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
