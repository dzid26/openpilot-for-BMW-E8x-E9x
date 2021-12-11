import struct
from PyTrinamic.TMCL import TMCL_Command, TMCL_Request
# from  PyTrinamic.modules.TMCM_1270 import _APs
from selfdrive.car import make_can_msg


class _APs:
    TargetPosition                 = 0
    ActualPosition                 = 1
    TargetVelocity                 = 2
    ActualVelocity                 = 3
    MaxVelocity                    = 4
    MaxAcceleration                = 5
    MaxCurrent                     = 6
    StandbyCurrent                 = 7
    PositionReachedFlag            = 8
    HomeSwitch                     = 9
    RightEndstop                   = 10
    LeftEndstop                    = 11
    AutomaticRightStop             = 12
    AutomaticLeftStop              = 13
    swapSwitchInputs               = 14
    A1                             = 15
    V1                             = 16
    MaxDeceleration                = 17
    D1                             = 18
    StartVelocity                  = 19
    StopVelocity                   = 20
    RampWaitTime                   = 21
    THIGH                          = 22
    VDCMIN                         = 23
    rightSwitchPolarity            = 24
    leftSwitchPolarity             = 25
    softstop                       = 26
    HighSpeedChopperMode           = 27
    HighSpeedFullstepMode          = 28
    MeasuredSpeed                  = 29
    PowerDownRamp                  = 31
    RelativePositioningOptionCode  = 127
    MicrostepResolution            = 140
    ChopperBlankTime               = 162
    ConstantTOffMode               = 163
    DisableFastDecayComparator     = 164
    ChopperHysteresisEnd           = 165
    ChopperHysteresisStart         = 166
    TOff                           = 167
    SEIMIN                         = 168
    SECDS                          = 169
    smartEnergyHysteresis          = 170
    SECUS                          = 171
    smartEnergyHysteresisStart     = 172
    SG2FilterEnable                = 173
    SG2Threshold                   = 174
    ShortToGroundProtection        = 177
    VSense                         = 179
    smartEnergyActualCurrent       = 180
    smartEnergyStallVelocity       = 181
    smartEnergyThresholdSpeed      = 182
    RandomTOffMode                 = 184
    ChopperSynchronization         = 185
    PWMThresholdSpeed              = 186
    PWMGrad                        = 187
    PWMAmplitude                   = 188
    PWMScale                       = 189
    pwmMode                        = 190
    PWMFrequency                   = 191
    PWMAutoscale                   = 192
    ReferenceSearchMode            = 193
    ReferenceSearchSpeed           = 194
    RefSwitchSpeed                 = 195
    RightLimitSwitchPosition       = 196
    LastReferencePosition          = 197
    encoderMode                    = 201
    MotorFullStepResolution        = 202
    pwmSymmetric                   = 203
    FreewheelingMode               = 204
    LoadValue                      = 206
    extendedErrorFlags             = 207
    DrvStatusFlags                 = 208
    EncoderPosition                = 209
    EncoderResolution              = 210
    max_EncoderDeviation           = 212
    PowerDownDelay                 = 214
    UnitMode                       = 255
    CurrentStepping                = 0

class _GPs:

    CANBitrate                    = 69
    CANSendId                     = 70
    CANReceiveId                  = 71
    CANSecondaryId                = 72
    autoStartMode                 = 77
    protectionMode                = 81
    eepromCoordinateStore         = 84
    zeroUserVariables             = 85
    applicationStatus             = 128
    programCounter                = 130
    lastTmclError                 = 131
    tickTimer                     = 132
    randomNumber                  = 133

def create_steer_command(packer, steer_angle_delta):
    """Creates a CAN message for the Trinamic actuator Steer Command"""
    requestPack = TMCL_Request(252, TMCL_Command.MVP, 0x01, 0x00, int(steer_angle_delta))
    values = {
        "Command": TMCL_Command.MVP,
        "CommandType": 1, #1 means relative position, aka "move by"
        "MotorBank": 0,
        "Value": steer_angle_delta,
    }
    # requestPack.dump()
    return make_can_msg(252, requestPack.toBuffer()[1:], 2)  #bus 2 - Trinamic CANbus
    # return packer.make_can_msg("TMCL_actuatorRequest", 0, values)

def create_steer_current_command(packer, steer_current, max_curr):
    """Creates a CAN message for the Trinamic actuator Steer Command"""
    curr_raw = int(round(steer_current / max_curr * 255)) & 0xFF  # max_curr corresponds to 255
    requestPack = TMCL_Request(253, TMCL_Command.SAP, _APs.MaxCurrent, 0x00, curr_raw)
    # print("Current: " + str(curr_raw))
    return make_can_msg(253, requestPack.toBuffer()[1:], 2)  #bus 2 - Trinamic CANbus

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

