import struct
from selfdrive.car import make_can_msg

def create_steer_command(packer, steer_angle_chg):
    values = {"steer_angle_chg": steer_angle_chg}
    return packer.make_can_msg("Dummy", 1, values)

def create_accel_command(packer, action, bus, frame):
    cnt2 = frame % 15 #cnt2 counts from 0 to 14
    if action == "plus1":
        cnt1_offset = 2
        values = {"setMe_0xFC": 0xFC,
                  "requests_0xF": 0xF,
                  "plus1mph_request": 1,
                  "notCancel_0xF": 0xF,
                  "Counter1": (cnt2 - cnt1_offset) % 16, #BMW made counters weird - cnt1 counts to 15 but skips a step when cnt2 rolls to 0
                  "Counter2": cnt2
                  }
    elif action == "minus1":
        cnt1_offset = -1
        values = {"setMe_0xFC": 0xFC,
                  "requests_0xF": 0xF,
                  "minus1mph_request": 1,
                   "notCancel_0xF": 0xF,
                  "Counter1": (cnt2 - cnt1_offset) % 16,
                  "Counter2": cnt2
                  }
    elif action == "cancel":
        cnt1_offset = 2
        values = {"setMe_0xFC": 0xFC,
                  "requests_0xF": 0xF,
                  "Cancel_request_up_stalk": 1,
                  "notCancel_0xF": 0x0,
                  "Counter1": (cnt2 - cnt1_offset) % 16,
                  "Counter2": cnt2
                  }
    # bus 0 - send on PT-CAN (JBBE <-> DME) works for V0540 only
    # bus 1 - send on F-CAN (SZL <-> DSC) works for both VO544 and V0540
    return packer.make_can_msg("CruiseControl", bus, values)

