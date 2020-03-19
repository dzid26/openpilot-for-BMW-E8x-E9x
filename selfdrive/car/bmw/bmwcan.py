import struct
from PyTrinamic.TMCL import TMCL_Command, TMCL_Request
# from  PyTrinamic.modules.TMCM_1270 import _APs
from selfdrive.car import make_can_msg

def create_steer_command(packer, steer_angle_chg):
    """Creates a CAN message for the Trinamic actuator Steer Command"""
    requestPack = TMCL_Request(0xFC, TMCL_Command.MVP, 0x01, 0x00, int(steer_angle_chg))
    values = {
        "Command": TMCL_Command.MVP,
        "CommandType": 1, #1 means relative position, aka "move by"
        "MotorBank": 0,
        "Value": steer_angle_chg,
    }
    # requestPack.dump()
    return make_can_msg(0xFC, requestPack.toBuffer()[1:], 2)
    # return packer.make_can_msg("TMCL_actuatorRequest", 0, values)


def create_accel_command(packer, action):
    count = 1  # TODO maybe needs to be replaced with rolling counter -  consider overflows if rolling counter used
    if action == "plus1":
        cnt2_offset = 2
        values = {"setMe_0xFC": 0xFC,
                  "requests_0xF": 0xF,
                  "plus1mph_request": 1,
                  "notCancel_0xF": 0xF,
                  "Counter1": count,
                  "Counter2": count + cnt2_offset
                  }
    elif action == "minus1":
        cnt2_offset = -1
        values = {"setMe_0xFC": 0xFC,
                  "requests_0xF": 0xF,
                  "minus1mph_request": 1,
                   "notCancel_0xF": 0xF,
                  "Counter1": count,
                  "Counter2": count + cnt2_offset
                  }
    elif action == "cancel":
        cnt2_offset = 2
        values = {"setMe_0xFC": 0xFC,
                  "requests_0xF": 0xF,
                  "Cancel_request_up_stalk": 1,
                  "notCancel_0xF": 0x0,
                  "Counter1": count,
                  "Counter2": count + cnt2_offset
                  }
    return packer.make_can_msg("CruiseControl", 1, values)  #bus 1 - send on F-CAN (SZL <-> DSC) should work for both VO544 and V0540

