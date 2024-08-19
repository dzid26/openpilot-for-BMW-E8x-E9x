#!/usr/bin/env python3
from cereal import car
from openpilot.selfdrive.car import create_button_events
from openpilot.selfdrive.car.helpers import interp
from openpilot.selfdrive.car.conversions import Conversions as CV
from openpilot.selfdrive.car import get_safety_config
from openpilot.selfdrive.car.bmw.values import CAR, CanBus, BmwFlags, CarControllerParams
from openpilot.selfdrive.car.interfaces import CarInterfaceBase
from openpilot.selfdrive.controls.lib.vehicle_model import VehicleModel

ButtonType = car.CarState.ButtonEvent.Type
EventName = car.CarEvent.EventName
TransmissionType = car.CarParams.TransmissionType
GearShifter = car.CarState.GearShifter

# certain driver intervention can be distinguished from maximum estimated wheel turning force
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
    self.VM = VehicleModel(CP)  # for the yawRate

    self.cp_F = self.CS.get_F_can_parser(CP)
    self.can_parsers.append(self.cp_F)
    self.cp_aux = self.CS.get_actuator_can_parser(CP)
    self.can_parsers.append(self.cp_aux)

  @staticmethod
  # servotronic is a bit more lighter in general and especially at low speeds https://www.spoolstreet.com/threads/servotronic-on-a-335i.1400/page-13#post-117705
  def get_steer_feedforward_servotronic(desired_angle, v_ego): # accounts for steering rack ratio and/or caster nonlinearities https://www.spoolstreet.com/threads/servotronic-on-a-335i.1400/page-15#post-131271
    hold_BP = [-40.0, -6.0, -4.0, -3.0, -2.0, -1.0, -0.5,  0.5,  1.0,  2.0,  3.0,  4.0,  6.0, 40.0]
    hold_V  = [-12.0, -5.7, -5.0, -4.5, -4.0, -3.3, -2.5,  2.5,  3.3,  4.0,  4.5,  5.0,  5.7, 12.0]
    rat = interp(desired_angle, hold_BP, hold_V)
    return desired_angle * rat # todo add speed component

  @staticmethod
  def get_steer_feedforward(desired_angle, v_ego):
    hold_BP = [-40.0, -6.0, -4.0, -3.0, -2.0, -1.0, -0.5,  0.5,  1.0,  2.0,  3.0,  4.0,  6.0, 40.0]
    hold_V  = [-12.0, -5.7, -5.0, -4.5, -4.0, -3.3, -2.5,  2.5,  3.3,  4.0,  4.5,  5.0,  5.7, 12.0]
    rat = interp(desired_angle, hold_BP, hold_V)
    return desired_angle * rat # todo add speed component

  def get_steer_feedforward_function(self):
    if self.CP.flags & BmwFlags.SERVOTRONIC:
      return self.get_steer_feedforward_servotronic
    else:
      return self.get_steer_feedforward

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs):
    if 0x22F in fingerprint[CanBus.SERVO_CAN]:   # Enigne controls speed and reports cruise control status
      ret.flags |= BmwFlags.STEPPER_SERVO_CAN.value

    ret.openpilotLongitudinalControl = True
    ret.pcmCruise = False # use OP speed tracking because we control speed using stock cruise speed setpoint or stock cruise is disabled

    ret.autoResumeSng = False
    if 0x200 in fingerprint[CanBus.PT_CAN]:   # Enigne controls speed and reports cruise control status
      ret.flags |= BmwFlags.NORMAL_CRUISE_CONTROL.value # openpilot will inject cruise stalk +/- requests
    elif 0x193 in fingerprint[CanBus.PT_CAN]:   # either DSC or LDM reports cruise control status
      if 0x0D5 not in fingerprint[CanBus.PT_CAN]:                                   # DSC itself applies brakes
        ret.flags |= BmwFlags.DYNAMIC_CRUISE_CONTROL.value # openpilot will inject cruise stalk +/- requests on F-CAN
      else: # LDM sends brake commands
        ret.flags |= BmwFlags.ACTIVE_CRUISE_CONTROL_NO_ACC.value # openpilot will switch between OP and LDM
        ret.autoResumeSng = True #! hopefully
    else: # DSC/DME not sending cruise status and LDM not present - openpilot will be the only requester
      ret.flags |= BmwFlags.ACTIVE_CRUISE_CONTROL_NO_LDM.value
      ret.autoResumeSng = True #! hopefully

    if 0xb8 in fingerprint[CanBus.PT_CAN] or 0xb5 in fingerprint[CanBus.PT_CAN]: # transmission: engine torque requests
      ret.transmissionType = TransmissionType.automatic
    else:
      ret.transmissionType = TransmissionType.manual

    # Detect all wheel drive BMW E90 XI
    if 0xbc in fingerprint[CanBus.PT_CAN]: # XI has a transfer case
      ret.steerRatio = 18.5 # XI has slower steering rack

    # todo enable is done by stock CC, so probably delete this
    # is_metric = Params().get("IsMetric", encoding='utf8') == "1"
    # if ret.flags & BmwFlags.DYNAMIC_CRUISE_CONTROL:  # DCC imperial has higher threshold
    #   ret.minEnableSpeed = 30. * CV.KPH_TO_MS if is_metric else 20. * CV.MPH_TO_MS
    # if ret.flags & BmwFlags.NORMAL_CRUISE_CONTROL:
    #   ret.minEnableSpeed = 18. * CV.MPH_TO_MS

    ret.carName = "bmw"
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.bmw)]
    ret.radarUnavailable = True

    ret.steerControlType = car.CarParams.SteerControlType.torque
    ret.steerActuatorDelay = 0.15
    ret.steerLimitTimer = 0.4
    ret.lateralTuning.init('pid')
    ret.lateralTuning.pid.kpBP = [5.5, 30.]
    ret.lateralTuning.pid.kiBP = [5.5, 30.]
    ret.lateralTuning.pid.kpV = [0.5 / CarControllerParams.STEER_MAX, 3.0 / CarControllerParams.STEER_MAX]
    ret.lateralTuning.pid.kiV = [0.0 / CarControllerParams.STEER_MAX, 0.0 / CarControllerParams.STEER_MAX]
    ret.lateralTuning.pid.kf =   1.0 / CarControllerParams.STEER_MAX # get_steer_feedforward_function

    ret.longitudinalTuning.kpBP = [0.]
    ret.longitudinalTuning.kpV = [.1]
    ret.longitudinalTuning.kiBP = [0.]
    ret.longitudinalTuning.kiV = [0.]

    ret.centerToFront = ret.wheelbase * 0.44

    ret.startAccel = 0.0
    print("Controller: " + ret.lateralTuning.which())

    # has_servotronic = False
    # for fw in car_fw:  # todo check JBBF firmware for $216A
    #   if fw.ecu == "eps" and b"," in fw.fwVersion:
    #     has_servotronic = True

    return ret

  def _update(self, c):
    # ******************* do can recv *******************
    ret = self.CS.update(self.cp, self.cp_F, self.cp_aux)

    ret.yawRate = self.VM.yaw_rate(ret.steeringAngleDeg * CV.DEG_TO_RAD, ret.vEgo, 0) # todo use canbus signal

    ret.buttonEvents = [
      *create_button_events(self.CS.cruise_plus, self.CS.prev_cruise_plus, {1: ButtonType.accelCruise}),
      *create_button_events(self.CS.cruise_minus, self.CS.prev_cruise_minus, {1: ButtonType.decelCruise}),
      *create_button_events(self.CS.cruise_resume, self.CS.prev_cruise_resume, {1: ButtonType.resumeCruise}),
      *create_button_events(self.CS.cruise_cancel, self.CS.prev_cruise_cancel, {1: ButtonType.cancel}),
      *create_button_events(self.CS.otherButtons, not self.CS.otherButtons, {1: ButtonType.altButton1}),
    ]

    # events
    events = self.create_common_events(ret, pcm_enable=True)
    if ret.vEgo < self.CP.minEnableSpeed and self.CP.openpilotLongitudinalControl:
      events.add(EventName.speedTooLow)
      if ret.vEgo < self.CP.minEnableSpeed - 1:
        # while in standstill, send a user alert
        events.add(EventName.manualRestart)

    # *** disable/enable OpenPilot on resume button press ***
    resume_rising_edge = self.CS.cruise_resume and not self.CS.prev_cruise_resume and ret.cruiseState.enabled and self.CS.out.cruiseState.enabled
    if resume_rising_edge and c.enabled:
      events.add(EventName.buttonCancel)
    # when in cruise control, press resume to resume OpenPilot
    elif resume_rising_edge and not c.enabled:
      events.add(EventName.buttonEnable)
    # if self.CS.gas_kickdown: # todo do we want disable on kickdown?
    #   events.add(EventName.pedalPressed', [ET.NO_ENTRY, ET.USER_DISABLE]))

    ret.events = events.to_msg()

    return ret
