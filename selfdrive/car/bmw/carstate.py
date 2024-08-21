from cereal import car
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from openpilot.selfdrive.car.conversions import Conversions as CV
from openpilot.selfdrive.car.interfaces import CarStateBase
from openpilot.selfdrive.car.bmw.values import DBC, CanBus, BmwFlags
from openpilot.common.params import Params

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    self.shifter_values = can_define.dv["TransmissionDataDisplay"]['ShiftLeverPosition']
    self.steer_angle_delta = 0.
    self.gas_kickdown = False

    self.is_metric = Params().get("IsMetric", encoding='utf8') == "1"   #todo set is_metric in _get_params somehow
    self.cruise_stalk_plus = False
    self.cruise_stalk_minus = False
    self.cruise_stalk_plus5 = False
    self.cruise_stalk_minus5 = False
    self.cruise_stalk_resume = False
    self.cruise_stalk_cancel = False
    self.cruise_stalk_cancel_up = False
    self.cruise_stalk_cancel_dn = False
    self.prev_cruise_plus = self.cruise_stalk_plus
    self.prev_cruise_minus = self.cruise_stalk_minus
    self.prev_cruise_plus5 = self.cruise_stalk_plus5
    self.prev_cruise_minus5 = self.cruise_stalk_minus5
    self.prev_cruise_resume = self.cruise_stalk_resume
    self.prev_cruise_cancel = self.cruise_stalk_cancel
    self.prev_cruise_cancelUpStalk = self.cruise_stalk_cancel_up
    self.prev_cruise_cancelDnStalk = self.cruise_stalk_cancel_dn

    self.right_blinker_pressed = False
    self.left_blinker_pressed = False
    self.otherButtons = False
    self.prev_gasPressed = False
    self.dtc_mode = False

  def update(self, cp_PT, cp_F, cp_aux):
    # set these prev states at the beginning because they are used outside the update()
    self.prev_cruise_plus = self.cruise_stalk_plus
    self.prev_cruise_minus = self.cruise_stalk_minus
    self.prev_cruise_plus5 = self.cruise_stalk_plus5
    self.prev_cruise_minus5 = self.cruise_stalk_minus5
    self.prev_cruise_resume = self.cruise_stalk_resume
    self.prev_cruise_cancel = self.cruise_stalk_cancel
    self.prev_cruise_cancelUpStalk = self.cruise_stalk_cancel_up
    self.prev_cruise_cancelDnStalk = self.cruise_stalk_cancel_dn

    ret = car.CarState.new_message()

    ret.doorOpen = False # not any([cp.vl["SEATS_DOORS"]['DOOR_OPEN_FL'], cp.vl["SEATS_DOORS"]['DOOR_OPEN_FR']
    ret.seatbeltUnlatched = False # not cp.vl["SEATS_DOORS"]['SEATBELT_DRIVER_UNLATCHED']

    ret.brakePressed = cp_PT.vl["EngineAndBrake"]['BrakePressed'] != 0
    ret.parkingBrake = cp_PT.vl["Status_contact_handbrake"]["Handbrake_pulled_up"] != 0
    ret.gas = cp_PT.vl['AccPedal']["AcceleratorPedalPercentage"]
    ret.gasPressed = cp_PT.vl['AccPedal']["AcceleratorPedalPressed"] != 0
    self.gas_kickdown = cp_PT.vl['AccPedal']["KickDownPressed"] != 0 #BMW has kickdown button at the bottom of the pedal

    ret.wheelSpeeds = self.get_wheel_speeds(
      cp_PT.vl["WheelSpeeds"]["Wheel_FL"],
      cp_PT.vl["WheelSpeeds"]["Wheel_FR"],
      cp_PT.vl["WheelSpeeds"]["Wheel_RL"],
      cp_PT.vl["WheelSpeeds"]["Wheel_RR"],
    )
    ret.vEgoRaw = cp_PT.vl['Speed']["VehicleSpeed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = not cp_PT.vl['Speed']["MovingForward"] and not cp_PT.vl['Speed']["MovingReverse"]

    ret.steeringRateDeg = (cp_PT.vl["SteeringWheelAngle"]['SteeringSpeed'])
    can_gear = int(cp_PT.vl["TransmissionDataDisplay"]['ShiftLeverPosition'])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))
    blinker_on = cp_PT.vl["TurnSignals"]['TurnSignalActive'] != 0 and cp_PT.vl["TurnSignals"]['TurnSignalIdle'] == 0
    ret.leftBlinker = blinker_on and cp_PT.vl["TurnSignals"]['LeftTurn'] !=0   # blinking
    ret.rightBlinker = blinker_on and cp_PT.vl["TurnSignals"]['RightTurn'] !=0   # blinking
    self.right_blinker_pressed = not blinker_on and cp_PT.vl["TurnSignals"]['RightTurn'] != 0
    self.left_blinker_pressed = not blinker_on and cp_PT.vl["TurnSignals"]['LeftTurn'] != 0

    self.dtc_mode = cp_PT.vl['StatusDSC_KCAN']['DTC_on'] != 0 # drifty traction control ;)

    self.otherButtons = \
      cp_PT.vl["SteeringButtons"]['Volume_DOWN'] !=0  or cp_PT.vl["SteeringButtons"]['Volume_UP'] !=0  or \
      cp_PT.vl["SteeringButtons"]['Previous_down'] !=0  or cp_PT.vl["SteeringButtons"]['Next_up'] !=0 or \
      self.prev_gasPressed and not ret.gasPressed # treat gas pedal tap as a button - button events indicate driver engagement - useful if face not visible
      # TODO: add other buttons (lights, gear, DTC, etc)

    # emulate driver steering torque - allows lane change assist on blinker hold
    ret.steeringPressed = ret.gasPressed # E-series doesn't have torque sensor - tapping the gas while using a blinker confirms driver intention to change lane
    if ret.steeringPressed and ret.leftBlinker:
      ret.steeringTorque = 1
    elif ret.steeringPressed and  ret.rightBlinker:
      ret.steeringTorque = -1
    else:
      ret.steeringTorque = 0

    ret.espDisabled = cp_PT.vl['StatusDSC_KCAN']['DSC_full_off'] != 0
    ret.cruiseState.available = not ret.espDisabled  #cruise not available when DSC fully off
    ret.cruiseState.nonAdaptive = False # bmw doesn't have a switch

    # todo # *** determine is_metric based speed target vs actual speed ***
    # if ret.cruiseState.enabled:
    #   if abs(ret.cruiseState.speed / ret.vEgo - CV.MS_TO_KPH) < 0.3:
    #     self.is_metric = True
    #   elif abs(ret.cruiseState.speed / ret.vEgo - CV.MS_TO_MPH) < 0.3:
    #     self.is_metric = False

    cruiseControlStalkMsg = cp_PT.vl["CruiseControlStalk"]
    if self.CP.flags & BmwFlags.DYNAMIC_CRUISE_CONTROL:
      ret.steeringAngleDeg = (cp_F.vl['SteeringWheelAngle_DSC']['SteeringPosition'])  # slightly quicker on F-CAN TODO find the factor and put in DBC
      ret.cruiseState.speed = cp_PT.vl["DynamicCruiseControlStatus"]['CruiseControlSetpointSpeed'] * (CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS)
      ret.cruiseState.enabled = cp_PT.vl["DynamicCruiseControlStatus"]['CruiseActive'] != 0
      # DCC implies that cruise control is done on F-CAN
      # If we are sending on F-can, we also need to read on F-can to differentiate our messages from car messages
      cruiseControlStalkMsg = cp_F.vl["CruiseControlStalk"]
    elif self.CP.flags & BmwFlags.NORMAL_CRUISE_CONTROL:
      ret.steeringAngleDeg = (cp_PT.vl['SteeringWheelAngle']['SteeringPosition'])
      ret.cruiseState.speed = cp_PT.vl["CruiseControlStatus"]['CruiseControlSetpointSpeed'] * (CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS)
      ret.cruiseState.enabled = cp_PT.vl["CruiseControlStatus"]['CruiseCoontrolActiveFlag'] != 0

    self.cruise_stalk_plus = cruiseControlStalkMsg['plus1'] != 0
    self.cruise_stalk_minus = cruiseControlStalkMsg['minus1'] != 0
    self.cruise_stalk_plus5 = cruiseControlStalkMsg['plus5'] != 0
    self.cruise_stalk_minus5 = cruiseControlStalkMsg['minus5'] != 0
    self.cruise_stalk_resume = cruiseControlStalkMsg['resume'] != 0
    self.cruise_stalk_cancel = cruiseControlStalkMsg['cancel'] != 0
    self.cruise_stalk_cancel_up = cruiseControlStalkMsg['cancel_lever_up'] != 0
    self.cruise_stalk_counter = cruiseControlStalkMsg['Counter_404']
    self.cruise_stalk_cancel_dn = self.cruise_stalk_cancel and not self.cruise_stalk_cancel_up


    ret.genericToggle = self.dtc_mode

    if self.CP.flags & BmwFlags.STEPPER_SERVO_CAN:
      ret.steeringTorqueEps =  cp_aux.vl['STEERING_STATUS']['STEERING_TORQUE']
      self.steer_angle_delta = cp_aux.vl['STEERING_STATUS']['STEERING_ANGLE']
      ret.steerFaultTemporary = int(cp_aux.vl['STEERING_STATUS']['CONTROL_STATUS']) & 0x4 != 0

    self.prev_gasPressed = ret.gasPressed
    return ret

  @staticmethod
  def get_can_parser(CP): #PT-CAN
    messages = [ # message, frequency
      ("EngineAndBrake", 100),
      ("TransmissionDataDisplay", 5),
      ("AccPedal", 100),
      ("Speed", 50),
      ("SteeringWheelAngle", 100),
      ("TurnSignals", 0),
      ("SteeringButtons", 0),
      ("WheelSpeeds", 50), # 100 on F-CAN
      ("CruiseControlStalk", 5),
      ("StatusDSC_KCAN", 50),
      ("Status_contact_handbrake", 0),
      ("TerminalStatus", 10),
    ]

    if CP.flags & BmwFlags.DYNAMIC_CRUISE_CONTROL:
      messages.append(("DynamicCruiseControlStatus", 5))
    if CP.flags & BmwFlags.NORMAL_CRUISE_CONTROL:
      messages.append(("CruiseControlStatus", 5))

    return CANParser(DBC[CP.carFingerprint]['pt'], messages, CanBus.PT_CAN)  # 0: PT-CAN

  @staticmethod # $540 vehicle option could use just PT_CAN, but $544 requires sending and receiving cruise commands on F-CAN. Use F-can. Works for both options
  def get_F_can_parser(CP):
    if CP.flags & BmwFlags.DYNAMIC_CRUISE_CONTROL:
      messages = [ # message, frequency
      ("SteeringWheelAngle_DSC", 100),
      ("CruiseControlStalk",  5),
      ]
    else:
      messages = []

    return CANParser(DBC[CP.carFingerprint]['pt'], messages, CanBus.F_CAN)

  @staticmethod
  def get_actuator_can_parser(CP):
    if CP.flags & BmwFlags.STEPPER_SERVO_CAN:
      messages = [ # message, frequency
      ("STEERING_STATUS", 100),
      ]
    else:
      messages = []
    return CANParser('ocelot_controls', messages, CanBus.SERVO_CAN)
