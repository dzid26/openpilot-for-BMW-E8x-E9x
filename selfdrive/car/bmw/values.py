from dataclasses import dataclass, field
from enum import IntFlag
from openpilot.selfdrive.car import Platforms, CarSpecs, CarDocs, PlatformConfig, dbc_dict, DbcDict, STD_CARGO_KG
from openpilot.selfdrive.car.conversions import Conversions as CV

# Steer torque limits
class CarControllerParams: #controls running @ 100hz
  STEER_MAX = 12  # Nm
  STEER_DELTA_UP = 10 / 100       # 10Nm/s
  STEER_DELTA_DOWN = 1000 / 100     # 10Nm/sample - no limit
  STEER_ERROR_MAX = 999     # max delta between torque cmd and torque motor

  # STEER_BACKLASH = 1 #deg
  def __init__(self, CP):
    pass

class BmwFlags(IntFlag):
  # Detected Flags
  STEPPER_SERVO_CAN = 2 ** 0
  NORMAL_CRUISE_CONTROL = 2 ** 1          # CC  $540
  DYNAMIC_CRUISE_CONTROL = 2 ** 2         # DCC $544
  ACTIVE_CRUISE_CONTROL = 2 ** 3          # ACC $541 - LDM and ACC sensor - #! not supported
  ACTIVE_CRUISE_CONTROL_NO_ACC = 2 ** 4   # no ACC module - DSC, DME, KOMBI coded to $541, LDM coded to $544
  ACTIVE_CRUISE_CONTROL_NO_LDM = 2 ** 5   # no LDM/ACC - DSC, DME, KOMBI coded to $541
  SERVOTRONIC = 2 ** 6                    # ServoTonic $216A - TODO: needs firmware query

class CanBus:
  PT_CAN =    0
  SERVO_CAN = 1 # required for steering
  F_CAN =     1 # required for DYNAMIC_CRUISE_CONTROL or optional for logging
  K_CAN =     2 # not used - only logging


@dataclass
class BmwPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict('bmw_e9x_e8x', None))


class CAR(Platforms):
  BMW_E82 = BmwPlatformConfig(
    [CarDocs("BMW E82 2007", "VO540, VO544, VO541")],
    CarSpecs(mass=3145. * CV.LB_TO_KG + STD_CARGO_KG, wheelbase=2.66, steerRatio=16.00)
  )
  BMW_E90 = BmwPlatformConfig(
    [CarDocs("BMW E90 2006", "VO540, VO544, VO541")],
    CarSpecs(mass=3300. * CV.LB_TO_KG + STD_CARGO_KG, wheelbase=2.76, steerRatio=16.00)
  )


DBC = CAR.create_dbc_map()
