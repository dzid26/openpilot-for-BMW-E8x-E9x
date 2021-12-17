from cereal import car
from common.kalman.simple_kalman import KF1D
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from selfdrive.config import Conversions as CV
from selfdrive.car.bmw.values import CAR, DBC, STEER_THRESHOLD, NO_DSU_CAR

GearShifter = car.CarState.GearShifter

def parse_gear_shifter(gear, vals):
  val_to_capnp = {'P': GearShifter.park, 'R': GearShifter.reverse, 'N': GearShifter.neutral,
                  'D': GearShifter.drive, 'B': GearShifter.brake}
  try:
    return val_to_capnp[vals[gear]]
  except KeyError:
    return GearShifter.drive  # TODO safety - should be "unknown"


def get_PT_can_parser(CP):
  signals = [  # signal name, message name, default value
    # sig_name, sig_address, default
    ("BrakePressed", "Brake", 0),
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
    # ("STEER_FRACTION", "STEER_ANGLE_SENSOR", 0),
    # ("GAS_RELEASED", "PCM_CRUISE", 0),
    # ("CRUISE_STATE", "PCM_CRUISE", 0),
    # ("MAIN_ON", "PCM_CRUISE_2", 0),
    # ("SET_SPEED", "PCM_CRUISE_2", 0),
    # ("LOW_SPEED_LOCKOUT", "PCM_CRUISE_2", 0),
    # ("STEER_TORQUE_DRIVER", "STEER_TORQUE_SENSOR", 0),
    # ("STEER_TORQUE_EPS", "STEER_TORQUE_SENSOR", 0),
    ("TurnSignalIdle", "TurnSignals", 0),
    ("TurnSignalActive", "TurnSignals", 0),
    ("RightTurn", "TurnSignals", 0),
    ("LeftTurn", "TurnSignals", 0),
    ("Volume_DOWN", "SteeringButtons", 0),
    ("Volume_UP", "SteeringButtons", 0),
    ("Wheel1", "WheelSpeeds", 0),
    ("Wheel2", "WheelSpeeds", 0),
    ("Wheel3", "WheelSpeeds", 0),
    ("Wheel4", "WheelSpeeds", 0),
    # ("IPAS_STATE", "EPS_STATUS", 1),
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
    ("Brake", 80),
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


def get_F_can_parser(CP): #540 vehicle option could get away with just PT_CAN, but vo544 requires sending, and thus receiving, cruise commands on F-CAN. F-can works for both options.
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


#def get_cam_can_parser(CP):

 # signals = [("FORCE", "PRE_COLLISION", 0), ("PRECOLLISION_ACTIVE", "PRE_COLLISION", 0)]

  # use steering message to check if panda is connected to frc
 # checks = [("STEERING_LKA", 42)]

 # return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 1) # 1: F-CAN, 2: PT-CAN


class CarState():
  def __init__(self, CP):

    self.CP = CP
    self.can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    self.shifter_values = self.can_define.dv["GearSelectorSwitch"]['Gear']
    self.left_blinker_on = 0
    self.right_blinker_on = 0
    self.angle_offset = 0.
    self.pcm_acc_active = False
    self.init_angle_offset = False

    self.cruise_plus = False
    self.cruise_minus = False
    self.cruise_plus5 = False
    self.cruise_minus5 = False
    self.cruise_resume = False
    self.cruise_cancel = False
    self.cruise_cancelUpStalk = False
    self.cruise_cancelDnStalk = False

    # initialize can parser
    self.car_fingerprint = CP.carFingerprint

    # vEgo kalman filter
    dt = 0.01
    # Q = np.matrix([[10.0, 0.0], [0.0, 100.0]])
    # R = 1e3
    self.v_ego_kf = KF1D(x0=[[0.0], [0.0]],
                         A=[[1.0, dt], [0.0, 1.0]],
                         C=[1.0, 0.0],
                         K=[[0.12287673], [0.29666309]])
    self.v_ego = 0.0

  def update(self, cp_PT, cp_F):
    # copy can_valid
    self.can_valid = cp_PT.can_valid

    # update prevs, update must run once per loop
    self.prev_left_blinker_on = self.left_blinker_on
    self.prev_right_blinker_on = self.right_blinker_on

    self.prev_cruise_plus = self.cruise_plus
    self.prev_cruise_minus = self.cruise_minus
    self.prev_cruise_plus5 = self.cruise_plus5
    self.prev_cruise_minus5 = self.cruise_minus5
    self.prev_cruise_resume = self.cruise_resume
    self.prev_cruise_cancel = self.cruise_cancel
    self.prev_cruise_cancelUpStalk = self.cruise_cancelUpStalk
    self.prev_cruise_cancelDnStalk = self.cruise_cancelDnStalk

    # self.door_all_closed = not any([cp.vl["SEATS_DOORS"]['DOOR_OPEN_FL'], cp.vl["SEATS_DOORS"]['DOOR_OPEN_FR'],
    #                                 cp.vl["SEATS_DOORS"]['DOOR_OPEN_RL'], cp.vl["SEATS_DOORS"]['DOOR_OPEN_RR']])
    # self.seatbelt = not cp.vl["SEATS_DOORS"]['SEATBELT_DRIVER_UNLATCHED']

    self.brake_pressed = cp_PT.vl["Brake"]['BrakePressed'] != 0
    self.pedal_gas = cp_PT.vl['AccPedal']["AcceleratorPedalPercentage"]
    self.car_gas = self.pedal_gas  # not used
    self.user_brake =   cp_PT.vl["Brake"]['BrakePressed'] # TODO find brake force?
    self.brake_lights = self.brake_pressed  # TODO find brake light?
    self.gas_pressed = cp_PT.vl['AccPedal']["AcceleratorPedalPressed"] != 0

    self.esp_disabled = 0; # cp.vl["ESP_CONTROL"]['TC_DISABLED']

    v_wheel = cp_PT.vl['Speed']["VehicleSpeed"] * CV.KPH_TO_MS
    # calc best v_ego estimate, by averaging two opposite corners
    self.v_wheel_fl = v_wheel  # cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_FL'] * CV.KPH_TO_MS #* speed_factor
    self.v_wheel_fr = v_wheel  # cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_FR'] * CV.KPH_TO_MS #* speed_factor
    self.v_wheel_rl = v_wheel  # cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_RL'] * CV.KPH_TO_MS #* speed_factor
    self.v_wheel_rr = v_wheel  # cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_RR'] * CV.KPH_TO_MS #* speed_factor


    # Kalman filter
    if abs(v_wheel - self.v_ego) > 2.0:  # Prevent large accelerations when car starts at non zero speed
      self.v_ego_kf.x = [[v_wheel], [0.0]]

    self.v_ego_raw = v_wheel
    v_ego_x = self.v_ego_kf.update(v_wheel)
    self.v_ego = float(v_ego_x[0])
    self.a_ego = float(v_ego_x[1])
    self.standstill = not v_wheel > 0.001

    self.angle_steers = (cp_F.vl['SteeringWheelAngle_DSC']['SteeringPosition']) # slightly quicker on F-CAN  TODO find the factor and put in DBC
    self.angle_steers_rate = (cp_PT.vl["SteeringWheelAngle"]['SteeringSpeed'])
    self.gear_shifter = parse_gear_shifter(3, self.car_fingerprint)
    self.left_blinker_on = cp_PT.vl["TurnSignals"]['TurnSignalActive'] and cp_PT.vl["TurnSignals"]['LeftTurn']
    self.right_blinker_on = cp_PT.vl["TurnSignals"]['TurnSignalActive'] and cp_PT.vl["TurnSignals"]['RightTurn']

    self.otherButons = cp_PT.vl["SteeringButtons"]['Volume_DOWN'] or cp_PT.vl["SteeringButtons"]['Volume_UP']

    self.cruise_plus = cp_F.vl["CruiseControl"]['plus1mph_request'] == 1
    self.cruise_minus = cp_F.vl["CruiseControl"]['minus1mph_request'] == 1
    self.cruise_plus5 = cp_F.vl["CruiseControl"]['plus5mph_request'] == 1
    self.cruise_minus5 = cp_F.vl["CruiseControl"]['minus5mph_request'] == 1
    self.cruise_resume = cp_F.vl["CruiseControl"]['Resume_request'] == 1
    self.cruise_cancel = cp_F.vl["CruiseControl"]['Cancel_request_up_or_down_stalk'] != 0
    self.cruise_cancelUpStalk = cp_F.vl["CruiseControl"]['Cancel_request_up_stalk'] != 0
    self.cruise_cancelDnStalk = self.cruise_cancel and not self.cruise_cancelUpStalk

    self.main_on = 1 #cruise control is always available
    if self.CP.carFingerprint in [CAR.E82_DCC, CAR.E90_DCC]:
      self.v_cruise_pcm = cp_PT.vl["DynamicCruiseControlStatus"]['CruiseControlSetpointSpeed']
      self.pcm_acc_active = cp_PT.vl["DynamicCruiseControlStatus"]['CruiseActive'] != 0
    elif self.CP.carFingerprint in [CAR.E82, CAR.E90]:
      self.v_cruise_pcm = cp_PT.vl["CruiseControlStatus"]['CruiseControlSetpointSpeed']
      self.pcm_acc_active = cp_PT.vl["CruiseControlStatus"]['CruiseCoontrolActiveFlag'] != 0

    can_gear = 0    # int(cp.vl["GEAR_PACKET"]['GEAR'])
    self.gear_shifter = parse_gear_shifter(can_gear, self.shifter_values)
    self.right_blinker = cp_PT.vl["TurnSignals"]['RightTurn'] != 0
    self.left_blinker = cp_PT.vl["TurnSignals"]['LeftTurn'] != 0
    self.blinker_on = cp_PT.vl["TurnSignals"]['TurnSignalActive'] == 2
    self.blinker_off = cp_PT.vl["TurnSignals"]['TurnSignalIdle'] == 2

    # 2 is standby, 10 is active. TODO: check that everything else is really a faulty state
    # self.steer_state = cp.vl["EPS_STATUS"]['LKA_STATE']
    self.steer_error = 0 #  cp.vl["EPS_STATUS"]['LKA_STATE'] not in [1, 5]
    # self.ipas_active = cp.vl['EPS_STATUS']['IPAS_STATE'] == 3
    self.brake_error = 0
    # self.steer_torque_driver = cp.vl["STEER_TORQUE_SENSOR"]['STEER_TORQUE_DRIVER']
    self.steer_torque_motor = 0 # cp.vl["STEER_TORQUE_SENSOR"]['STEER_TORQUE_EPS']
    # we could use the override bit from dbc, but it's triggered at too high torque values
    # self.steer_override = abs(self.steer_torque_driver) > STEER_THRESHOLD


    self.low_speed_lockout = False
   