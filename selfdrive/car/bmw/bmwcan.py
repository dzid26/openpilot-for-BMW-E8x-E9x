import crcmod
from opendbc.can.packer import CANPacker

openactuator_checksum = crcmod.mkCrcFun(0x1D7, initCrc=0xFF, rev=False, xorOut=0x00) # 0xeb (implicit notation) HD = 5 (detecting 4, correcting 2 bits)

def create_steer_command(mode, steer_delta, steer_tq, frame):
    """Creates a CAN message for the actuator ControlCommand"""
    packer = CANPacker('OpenActuator')
    values = {
        "Counter_CMD1": frame % 0xF,
        "MotorId_CMD1": 0x0,
        "ControlMode": mode,
        "PositionChange": steer_delta,
        "TorqueFeedforward": steer_tq,
        "TorqueCloseloopMax": 0., #TODO: generate signals for this
    }
    dat = packer.make_can_msg("ControlCmd1", 0, values)[2]
    # values["CRC8_CMD1"] = openactuator_checksum(dat[:7])
    values["CRC8_CMD1"] = (dat[0] + dat[1] + dat[2] + dat[3] + dat[4] + dat[5] + dat[6]) & 0xFF

    return packer.make_can_msg("ControlCmd1", 2, values) #bus 2 is the actuator CAN bus

    
def calc_checksum_4bit(work_data, msg_id): # 0x130
  checksum = msg_id
  for byte in work_data: #checksum is stripped from the dat
    checksum += byte     #add up all the bytes

  checksum = (checksum & 0xFF) + (checksum >> 8); #add upper and lower Bytes
  checksum &= 0xFF #throw away anything in upper Byte  

  checksum = (checksum & 0xF) + (checksum >> 4); #add first and second nibble
  checksum &= 0xF; #throw away anything in upper nibble
  return checksum

def calc_checksum_8bit(work_data, msg_id): # 0xb8 0x1a0 0x19e 0xaa 0xbf
  checksum = msg_id
  for byte in work_data: #checksum is stripped from the data
    checksum += byte     #add up all the bytes

  checksum = (checksum & 0xFF) + (checksum >> 8); #add upper and lower Bytes
  checksum &= 0xFF #throw away anything in upper Byte
  return checksum

def calc_checksum_cruise(work_data):# 0x194 this checksum is special - initilized with 0
  return calc_checksum_8bit(work_data, 0) 


def create_accel_command(packer, action, bus, frame):
    values = {
        "setMe_0xFC": 0xFC,
        "requests_0xF": 0xF,
        "Counter_404": frame % 15 #counts from 0 to 14
        }
    if action == "plus1":
        values["plus1mph_request"] = 1
    elif action == "minus1":
        values["minus1mph_request"] = 1
    elif action == "cancel":
        values["Cancel_request_up_stalk"] = 1

    dat = packer.make_can_msg("CruiseControl", bus, values)[2]
    values["Checksum_404"] = calc_checksum_cruise(dat)

    # bus 0 - send on PT-CAN (JBBE <-> DME) works for V0540 only
    # bus 1 - send on F-CAN (SZL <-> DSC) works for both VO544 and V0540
    return packer.make_can_msg("CruiseControl", bus, values)

