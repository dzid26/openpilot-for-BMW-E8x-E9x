from cereal import car
from common.numpy_fast import mean
from opendbc.can.can_define import CANDefine
from selfdrive.car.interfaces import CarStateBase
from opendbc.can.parser import CANParser
from selfdrive.config import Conversions as CV
from selfdrive.car.bmw.values import CAR, DBC, STEER_THRESHOLD, NO_DSU_CAR

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    self.shifter_values = can_define.dv["GearSelectorSwitch"]['Gear']
    self.angle_offset = 0.
    self.steer_warning = False
    self.low_speed_lockout = False

    self.cruise_plus = False
    self.cruise_minus = False
    self.cruise_plus5 = False
    self.cruise_minus5 = False
    self.cruise_resume = False
    self.cruise_cancel = False
    self.cruise_cancelUpStalk = False
    self.cruise_cancelDnStalk = False
    self.prev_cruise_plus = self.cruise_plus
    self.prev_cruise_minus = self.cruise_minus
    self.prev_cruise_plus5 = self.cruise_plus5
    self.prev_cruise_minus5 = self.cruise_minus5
    self.prev_cruise_resume = self.cruise_resume
    self.prev_cruise_cancel = self.cruise_cancel
    self.prev_cruise_cancelUpStalk = self.cruise_cancelUpStalk
    self.prev_cruise_cancelDnStalk = self.cruise_cancelDnStalk

  def update(self, cp_PT, cp_F):
    ret = car.CarState.new_message()

    ret.doorOpen = False # not any([cp.vl["SEATS_DOORS"]['DOOR_OPEN_FL'], cp.vl["SEATS_DOORS"]['DOOR_OPEN_FR']
    ret.seatbeltUnlatched = False # not cp.vl["SEATS_DOORS"]['SEATBELT_DRIVER_UNLATCHED']

    ret.brakePressed = cp_PT.vl["EngineAndBrake"]['BrakePressed'] != 0
    ret.gas = cp_PT.vl['AccPedal']["AcceleratorPedalPercentage"]
    ret.gasPressed = ret.gas > 20

    ret.espDisabled = False  # cp.vl["ESP_CONTROL"]['TC_DISABLED'] ==1

    ret.vEgoRaw = cp_PT.vl['Speed']["VehicleSpeed"] * CV.KPH_TO_MS
    ret.wheelSpeeds.fl = ret.vEgoRaw  # cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_FL'] * CV.KPH_TO_MS #* speed_factor
    ret.wheelSpeeds.fr = ret.vEgoRaw  # cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_FR'] * CV.KPH_TO_MS #* speed_factor
    ret.wheelSpeeds.fl = ret.vEgoRaw  # cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_RL'] * CV.KPH_TO_MS #* speed_factor
    ret.wheelSpeeds.fr = ret.vEgoRaw  # cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_RR'] * CV.KPH_TO_MS #* speed_factor
    ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])

    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    ret.standstill = not ret.vEgoRaw > 0.001

    ret.steeringAngle = (cp_F.vl['SteeringWheelAngle_DSC'][
      'SteeringPosition'])  # slightly quicker on F-CAN  TODO find the factor and put in DBC
    ret.steeringRate = (cp_PT.vl["SteeringWheelAngle"]['SteeringSpeed'])
    can_gear = 0  # int(cp.vl["GEAR_PACKET"]['GEAR'])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))
    ret.leftBlinker = cp_PT.vl["TurnSignals"]['TurnSignalActive'] !=0 and cp_PT.vl["TurnSignals"]['LeftTurn'] !=0   # blinking
    ret.rightBlinker = cp_PT.vl["TurnSignals"]['TurnSignalActive'] !=0  and cp_PT.vl["TurnSignals"]['RightTurn'] !=0   # blinking
    right_blinker_pressed = cp_PT.vl["TurnSignals"]['RightTurn'] != 0
    left_blinker_pressed = cp_PT.vl["TurnSignals"]['LeftTurn'] != 0
    blinker_on = cp_PT.vl["TurnSignals"]['TurnSignalActive'] == 2
    # self.blinker_off = cp_PT.vl["TurnSignals"]['TurnSignalIdle'] == 2

    ret.genericToggle = cp_PT.vl["SteeringButtons"]['Volume_DOWN'] !=0  or cp_PT.vl["SteeringButtons"]['Volume_UP'] !=0  or \
                        cp_PT.vl["SteeringButtons"]['Previous_down'] !=0  or cp_PT.vl["SteeringButtons"]['Next_up'] !=0
    ret.steeringTorque = 0
    # do lane change after releasing blinker stalk
    ret.steeringPressed = blinker_on and not right_blinker_pressed and not left_blinker_pressed

    self.cruise_plus = cp_F.vl["CruiseControl"]['plus1mph_request'] == 1
    self.cruise_minus = cp_F.vl["CruiseControl"]['minus1mph_request'] == 1
    self.cruise_plus5 = cp_F.vl["CruiseControl"]['plus5mph_request'] == 1
    self.cruise_minus5 = cp_F.vl["CruiseControl"]['minus5mph_request'] == 1
    self.cruise_resume = cp_F.vl["CruiseControl"]['Resume_request'] == 1
    self.cruise_cancel = cp_F.vl["CruiseControl"]['Cancel_request_up_or_down_stalk'] != 0
    self.cruise_cancelUpStalk = cp_F.vl["CruiseControl"]['Cancel_request_up_stalk'] != 0
    self.cruise_cancelDnStalk = self.cruise_cancel and not self.cruise_cancelUpStalk

    ret.cruiseState.available = True  # todo unless DCS (traction control) disabled
    if self.CP.carFingerprint in [CAR.E82_DCC, CAR.E90_DCC]:
      ret.cruiseState.speed = cp_PT.vl["DynamicCruiseControlStatus"]['CruiseControlSetpointSpeed']
      ret.cruiseState.enabled = cp_PT.vl["DynamicCruiseControlStatus"]['CruiseActive'] != 0
    elif self.CP.carFingerprint in [CAR.E82, CAR.E90]:
      ret.cruiseState.speed = cp_PT.vl["CruiseControlStatus"]['CruiseControlSetpointSpeed']
      ret.cruiseState.enabled = cp_PT.vl["CruiseControlStatus"]['CruiseCoontrolActiveFlag'] != 0
    ret.steeringTorqueEps = 0

    self.prev_cruise_plus = self.cruise_plus
    self.prev_cruise_minus = self.cruise_minus
    self.prev_cruise_plus5 = self.cruise_plus5
    self.prev_cruise_minus5 = self.cruise_minus5
    self.prev_cruise_resume = self.cruise_resume
    self.prev_cruise_cancel = self.cruise_cancel
    self.prev_cruise_cancelUpStalk = self.cruise_cancelUpStalk
    self.prev_cruise_cancelDnStalk = self.cruise_cancelDnStalk
    return ret

  @staticmethod
  def get_can_parser(CP): #PT-CAN
    signals = [  # signal name, message name, default value
      # sig_name, sig_address, default
      ("BrakePressed", "EngineAndBrake", 0),
      # ("GEAR", "GEAR_PACKET", 0),
      ("AcceleratorPedalPressed", "AccPedal", 0),
      ("AcceleratorPedalPercentage", "AccPedal", 0),
      ("VehicleSpeed", "Speed", 0),
      # ("DOOR_OPEN_FL", "SEATS_DOORS", 1),
      # ("DOOR_OPEN_FR", "SEATS_DOORS", 1),
      # ("DOOR_OPEN_RL", "SEATS_DOORS", 1),
      # ("DOOR_OPEN_RR", "SEATS_DOORS", 1),
      # ("SEATBELT_DRIVER_UNLATCHED", "SEATS_DOORS", 1),
      ("SteeringPosition", "SteeringWheelAngle", 0),
      ("SteeringSpeed", "SteeringWheelAngle", 0),
      ("TurnSignalIdle", "TurnSignals", 0),
      ("TurnSignalActive", "TurnSignals", 0),
      ("RightTurn", "TurnSignals", 0),
      ("LeftTurn", "TurnSignals", 0),
      ("Volume_DOWN", "SteeringButtons", 0),
      ("Volume_UP", "SteeringButtons", 0),
      ("Previous_down", "SteeringButtons", 0),
      ("Next_up", "SteeringButtons", 0),
      ("Wheel1", "WheelSpeeds", 0),
      ("Wheel2", "WheelSpeeds", 0),
      ("Wheel3", "WheelSpeeds", 0),
      ("Wheel4", "WheelSpeeds", 0),
      # ("BRAKE_LIGHTS_ACC", "ESP_CONTROL", 0),
      ("minus1mph_request", "CruiseControl", 0),
      ("plus1mph_request", "CruiseControl", 0),
      ("minus5mph_request", "CruiseControl", 0),
      ("plus5mph_request", "CruiseControl", 0),
      ("Resume_request", "CruiseControl", 0),
      ("Cancel_request_up_stalk", "CruiseControl", 0),
      ("Cancel_request_up_or_down_stalk", "CruiseControl", 0),
    ]

    checks = [  # refresh frequency Hz  TODO measure with PCAN
      ("EngineAndBrake", 80),
      ("AccPedal", 33),
      ("Speed", 80),
      # ("SteeringWheelAngle", 80),
    ]

    if CP.carFingerprint in [CAR.E82_DCC, CAR.E90_DCC]:
      signals.append(("CruiseActive", "DynamicCruiseControlStatus", 0))
      signals.append(("CruiseControlSetpointSpeed", "DynamicCruiseControlStatus",
                      252))  # 252 is what's read when CC is not engaged, so it seems like a good default
      checks.append(("DynamicCruiseControlStatus", 33))
    elif CP.carFingerprint in [CAR.E82, CAR.E90]:
      signals.append(("CruiseCoontrolActiveFlag", "CruiseControlStatus", 0))
      signals.append(("CruiseControlSetpointSpeed", "CruiseControlStatus", 0))
      checks.append(("CruiseControlStatus", 33))

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 2)  # 1: F-CAN, 2: PT-CAN

  @staticmethod
  def get_cam_can_parser( # F-CAN
    CP):  # 540 vehicle option could get away with just PT_CAN, but vo544 requires sending, and thus receiving, cruise commands on F-CAN. F-can works for both options.
    signals = [  # signal name, message name, default value
      # sig_name, sig_address, default
      ("SteeringPosition", "SteeringWheelAngle_DSC", 0),
      ("Wheel1", "WheelSpeeds", 0),
      ("Wheel2", "WheelSpeeds", 0),
      ("Wheel3", "WheelSpeeds", 0),
      ("Wheel4", "WheelSpeeds", 0),
      ("minus1mph_request", "CruiseControl", 0),
      ("plus1mph_request", "CruiseControl", 0),
      ("minus5mph_request", "CruiseControl", 0),
      ("plus5mph_request", "CruiseControl", 0),
      ("Resume_request", "CruiseControl", 0),
      ("Cancel_request_up_stalk", "CruiseControl", 0),
      ("Cancel_request_up_or_down_stalk", "CruiseControl", 0),
    ]

    checks = [  # refresh frequency Hz  TODO measure with PCAN
      ("SteeringWheelAngle_DSC", 80),
    ]
    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 1)  #
