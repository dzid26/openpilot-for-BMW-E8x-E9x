import struct
from selfdrive.car import make_can_msg

def create_steer_command(packer, steer_angle_chg):
    values = {"steer_angle_chg": steer_angle_chg}
    return packer.make_can_msg("Dummy", 1, values)


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

