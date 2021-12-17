from cereal import car
from common.numpy_fast import clip, interp
from selfdrive.car import apply_toyota_steer_torque_limits, create_gas_command, make_can_msg
from selfdrive.car.bmw.bmwcan import create_steer_command, create_accel_command
                                           #create_ui_command, create_fcw_command
from selfdrive.car.bmw.values import CAR, SteerLimitParams
from opendbc.can.packer import CANPacker
from selfdrive.config import Conversions as CV

VisualAlert = car.CarControl.HUDControl.VisualAlert
import time

# Accel limits
ACCEL_HYST_GAP = 0.02  # don't change accel command for small oscilalitons within this value
ACCEL_MAX = 4  # cruise control rapid clicking
ACCEL_SLOW = 3 # cruise control hold up
DECEL_SLOW = -2   # cruise control decrease speed slowly
DECEL_MIN = -6  # cruise control hold down
ACCEL_SCALE = max(ACCEL_MAX, -DECEL_MIN)


# Steer angle limits (tested at the Crows Landing track and considered ok)
ANGLE_MAX_BP = [0., 5.]
ANGLE_MAX_V = [510., 300.]
ANGLE_DELTA_BP = [0., 5., 15.]
ANGLE_DELTA_V = [5., .8, .15]     # windup limit
ANGLE_DELTA_VU = [5., 3.5, 0.4]   # unwind limit

TARGET_IDS = [0x340, 0x341, 0x342, 0x343, 0x344, 0x345,
              0x363, 0x364, 0x365, 0x370, 0x371, 0x372,
              0x373, 0x374, 0x375, 0x380, 0x381, 0x382,
              0x383]
def _current_time_millis():
  return int(round(time.time() * 1000))

def accel_hysteresis(accel, accel_steady, enabled):

  # for small accel oscillations within ACCEL_HYST_GAP, don't change the accel command
  if not enabled:
    # send 0 when disabled, otherwise acc faults
    accel_steady = 0.
  elif accel > accel_steady + ACCEL_HYST_GAP:
    accel_steady = accel - ACCEL_HYST_GAP
  elif accel < accel_steady - ACCEL_HYST_GAP:
    accel_steady = accel + ACCEL_HYST_GAP
  accel = accel_steady

  return accel, accel_steady


def process_hud_alert(hud_alert):
  # initialize to no alert
  steer = 0
  fcw = 0

  if hud_alert == VisualAlert.fcw:
    fcw = 1
  elif hud_alert == VisualAlert.steerRequired:
    steer = 1
  return steer, fcw


class CarController:
  def __init__(self, dbc_name, CP, VM):
    self.braking = False
    # redundant safety check with the board
    self.controls_allowed = True
    self.last_steer = 0
    self.last_angle = 0
    self.accel_steady = 0.
    self.alert_active = False
    self.last_standstill = False
    self.standstill_req = False
    self.angle_control = False
    self.last_frame_cruise_cmd_sent = 0
    self.last_type_cruise_cmd_sent = 0
    self.cruise_speed_prev = 0

    self.steer_angle_enabled = False
    self.ipas_reset_counter = 0
    self.last_fault_frame = -200
    self.steer_rate_limited = False
    self.cruise_bus = 0
    if CP.carFingerprint in [CAR.E82, CAR.E90]:
      self.cruise_bus = 0 #PT-CAN
    elif CP.carFingerprint in [CAR.E82_DCC, CAR.E90_DCC]:
      self.cruise_bus = 1 #F-CAN
    self.fake_ecus = set()

    self.packer = CANPacker(dbc_name)

  def update(self, control, CS, frame):
    requestedSpeed = control.cruiseControl.speedOverride
    current_time_ms = _current_time_millis()
    can_sends = []

    # *** compute control surfaces ***
    CC_cancel_cmd = 0
    # gas and brake

    # if not control.enabled and CS.out.pcm_acc_active!= 0:
    #   # send cruise control cancel command if openpilot  is disabled but cruise control is still on, or if the system can't be activated
    #   CC_cancel_cmd = 1
    # else:
    #   CC_cancel_cmd = control.cruiseControl.cancel
    # CC_cancel_cmd = control.cruiseControl.cancel

    buttons_pause_time = current_time_ms > (self.last_time_button_pressed + 100)

    if CC_cancel_cmd and buttons_pause_time:
      can_sends.append(create_accel_command(self.packer, "cancel"))
      self.last_time_button_pressed = current_time_ms
      print("cancel")
    elif ( ( requestedSpeed - CS.out.cruiseState.speed)> 0.5 *CV.MPH_TO_MS) and control.enabled  and buttons_pause_time:# err=desired-requested=70-67=3
      can_sends.append(create_accel_command(self.packer, "plus1"))
      self.last_time_button_pressed = current_time_ms
      print("+plus1      ", requestedSpeed, "    -    ",    CS.out.cruiseState.speed)
    elif (( CS.out.cruiseState.speed - requestedSpeed )> 0.6 *CV.MPH_TO_MS) and control.enabled and buttons_pause_time:# err=desired-requested=60-67=-7
      can_sends.append(create_accel_command(self.packer, "minus1"))
      self.last_time_button_pressed = current_time_ms
      print("-minus1      ", requestedSpeed, "    -    ",    CS.out.cruiseState.speed)


      # apply_accel =actuators.gas - actuators.brake
    # apply_accel, self.accel_steady = accel_hysteresis(apply_accel, self.accel_steady, enabled)
    # apply_accel = clip(apply_accel * ACCEL_SCALE, ACCEL_MIN, ACCEL_MAX)

    # steer torque
    new_steer = int(round(control.actuators.steer * SteerLimitParams.STEER_MAX))
    apply_steer = apply_toyota_steer_torque_limits(new_steer, self.last_steer, CS.out.steeringTorqueEps, SteerLimitParams)
    self.steer_rate_limited = new_steer != apply_steer

    # only cut torque when steer state is a known fault
    # if CS.out.steer_state in [9, 25]:
    #   self.last_fault_frame = frame

    # Cut steering for 2s after fault
    apply_hold_torque = 0
    if not control.enabled or (frame - self.last_fault_frame < 200):
      apply_steer_req = 0
    else:
      apply_steer_req = 1

    # steer angle
    if control.enabled:
      angle_lim = interp(CS.out.vEgo, ANGLE_MAX_BP, ANGLE_MAX_V)
      apply_angle = clip(control.actuators.steerAngle, -angle_lim, angle_lim)
      
      # windup slower
      if self.last_angle * apply_angle > 0. and abs(apply_angle) > abs(self.last_angle):
        angle_rate_lim = interp(CS.out.vEgo, ANGLE_DELTA_BP, ANGLE_DELTA_V)
      else:
        angle_rate_lim = interp(CS.out.vEgo, ANGLE_DELTA_BP, ANGLE_DELTA_VU)

      apply_angle = clip(apply_angle, self.last_angle - angle_rate_lim, self.last_angle + angle_rate_lim)

      can_sends.append(create_steer_command(self.packer, (CS.out.steeringAngle - apply_angle) / 1.8 * 256 * 19 * 25/12 ))
    else:
      apply_angle = CS.out.steeringAngle


    self.last_steer = apply_steer
    self.last_angle = apply_angle
    # self.last_accel = apply_accel
    self.last_standstill = CS.out.standstill




    # ui mesg is at 100Hz but we send asap if:
    # - there is something to display
    # - there is something to stop displaying
    alert_out = process_hud_alert(control.hudControl)
    steer, fcw = alert_out

    if (any(alert_out) and not self.alert_active) or \
       (not any(alert_out) and self.alert_active):
      send_ui = True
      self.alert_active = not self.alert_active
    else:
      send_ui = False


    #if frame % 100 == 0 and ECU.DSU in self.fake_ecus:
    #  can_sends.append(create_fcw_command(self.packer, fcw))

    #*** static msgs ***

    # for (addr, ecu, cars, bus, fr_step, vl) in STATIC_MSGS:
    #   if frame % fr_step == 0 and ecu in self.fake_ecus and self.car_fingerprint in cars:
    #    can_sends.append(make_can_msg(addr, vl, bus))

    return can_sends
