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
DECEL_MIN = -6.0  # cruise control hold down
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


# def ipas_state_transition(steer_angle_enabled, enabled, ipas_active, ipas_reset_counter):
#
#   if enabled and not steer_angle_enabled:
#     #ipas_reset_counter = max(0, ipas_reset_counter - 1)
#     #if ipas_reset_counter == 0:
#     #  steer_angle_enabled = True
#     #else:
#     #  steer_angle_enabled = False
#     #return steer_angle_enabled, ipas_reset_counter
#     return True, 0
#
#   elif enabled and steer_angle_enabled:
#     if steer_angle_enabled and not ipas_active:
#       ipas_reset_counter += 1
#     else:
#       ipas_reset_counter = 0
#     if ipas_reset_counter > 10:  # try every 0.1s
#       steer_angle_enabled = False
#     return steer_angle_enabled, ipas_reset_counter
#
#   else:
#     return False, 0


class CarController():
  def __init__(self, dbc_name, car_fingerprint, enable_camera, enable_dsu, enable_apg):
    self.braking = False
    # redundant safety check with the board
    self.controls_allowed = True
    self.last_steer = 0
    self.last_angle = 0
    self.accel_steady = 0.
    self.car_fingerprint = car_fingerprint
    self.alert_active = False
    self.last_standstill = False
    self.standstill_req = False
    self.angle_control = False
    self.last_time_button_pressed = 0

    self.steer_angle_enabled = False
    self.ipas_reset_counter = 0
    self.last_fault_frame = -200
    self.steer_rate_limited = False

    self.fake_ecus = set()
    # if enable_camera: self.fake_ecus.add(ECU.CAM)
    # if enable_dsu: self.fake_ecus.add(ECU.DSU)
    # if enable_apg: self.fake_ecus.add(ECU.APGS)

    self.packer = CANPacker(dbc_name)

  def update(self, control, CS, frame):
    requestedSpeed = control.cruiseControl.speedOverride * CV.MS_TO_MPH
    current_time_ms = _current_time_millis()
    print(control.enabled, "SpeedErr: ", requestedSpeed,  "actuator: ", control.actuators.gas, control.actuators.brake, control.actuators.steerAngle, CS.angle_steers)
    can_sends = []
    # test
    # can_sends.append(create_steer_command(self.packer, -5, -7777, 2))


    # *** compute control surfaces ***
    CC_cancel_cmd = 0
    # gas and brake

    # if not control.enabled and CS.pcm_acc_active!= 0:
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
    elif ( ( requestedSpeed - CS.v_cruise_pcm)> 0.5) and control.enabled  and buttons_pause_time:# err=desired-requested=70-67=3
      can_sends.append(create_accel_command(self.packer, "plus1"))
      self.last_time_button_pressed = current_time_ms
      print("+plus1      ", requestedSpeed, "    -    ",    CS.v_cruise_pcm)
    elif (( CS.v_cruise_pcm - requestedSpeed )> 0.6) and control.enabled and buttons_pause_time:# err=desired-requested=60-67=-7
      can_sends.append(create_accel_command(self.packer, "minus1"))
      self.last_time_button_pressed = current_time_ms
      print("-minus1      ", requestedSpeed, "    -    ",    CS.v_cruise_pcm)

    # if CC_cancel_cmd and buttons_pause_time:
    #   can_sends.append(create_accel_command(self.packer, "cancel"))
    #   self.last_time_button_pressed = current_time_ms
    #   print("cancel")
    # elif control.actuators.gas >0.5 and control.enabled and buttons_pause_time:  # err=desired-requested=70-67=3
    #   can_sends.append(create_accel_command(self.packer, "plus1"))
    #   self.last_time_button_pressed = current_time_ms
    #   print("+plus1      ", requestedSpeed, "    -    ", CS.v_cruise_pcm)
    # elif control.actuators.brake >0.5 and control.enabled and buttons_pause_time:  # err=desired-requested=60-67=-7
    #   can_sends.append(create_accel_command(self.packer, "minus1"))
    #   self.last_time_button_pressed = current_time_ms
    #   print("-minus1      ", requestedSpeed, "    -    ", CS.v_cruise_pcm)

      # apply_accel =actuators.gas - actuators.brake
    # apply_accel, self.accel_steady = accel_hysteresis(apply_accel, self.accel_steady, enabled)
    # apply_accel = clip(apply_accel * ACCEL_SCALE, ACCEL_MIN, ACCEL_MAX)

    # steer torque
    new_steer = int(round(control.actuators.steer * SteerLimitParams.STEER_MAX))
    apply_steer = apply_toyota_steer_torque_limits(new_steer, self.last_steer, CS.steer_torque_motor, SteerLimitParams)
    self.steer_rate_limited = new_steer != apply_steer

    # only cut torque when steer state is a known fault
    # if CS.steer_state in [9, 25]:
    #   self.last_fault_frame = frame

    # Cut steering for 2s after fault
    if not control.enabled or (frame - self.last_fault_frame < 200):
      apply_steer = 0
      apply_steer_req = 0
    else:
      apply_steer_req = 1

    # self.steer_angle_enabled, self.ipas_reset_counter = \
    #   ipas_state_transition(self.steer_angle_enabled, enabled, CS.ipas_active, self.ipas_reset_counter)
    #print("{0} {1} {2}".format(self.steer_angle_enabled, self.ipas_reset_counter, CS.ipas_active))

    # steer angle
    if control.enabled:
      apply_angle = control.actuators.steerAngle
      angle_lim = interp(CS.v_ego, ANGLE_MAX_BP, ANGLE_MAX_V)
      apply_angle = clip(apply_angle, -angle_lim, angle_lim)

      # windup slower
      if self.last_angle * apply_angle > 0. and abs(apply_angle) > abs(self.last_angle):
        angle_rate_lim = interp(CS.v_ego, ANGLE_DELTA_BP, ANGLE_DELTA_V)
      else:
        angle_rate_lim = interp(CS.v_ego, ANGLE_DELTA_BP, ANGLE_DELTA_VU)

      apply_angle = clip(apply_angle, self.last_angle - angle_rate_lim, self.last_angle + angle_rate_lim)

      can_sends.append(create_steer_command(self.packer, (CS.angle_steers - apply_angle) / 1.8 * 256 * 19 * 25/12 ))
    else:
      apply_angle = CS.angle_steers

    ## on entering standstill, send standstill request
    #if CS.standstill and not self.last_standstill:
    #  self.standstill_req = True
    #if CS.pcm_acc_active != 8:
    #  # pcm entered standstill or it's disabled
    #  self.standstill_req = False

    self.last_steer = apply_steer
    self.last_angle = apply_angle
    # self.last_accel = apply_accel
    self.last_standstill = CS.standstill


    #*** control msgs ***
    #print("steer {0} {1} {2} {3}".format(apply_steer, min_lim, max_lim, CS.steer_torque_motor)

      #if self.angle_control:
      #  can_sends.append(create_steer_command(self.packer, 0., 0, frame))
      #else:
      #  can_sends.append(create_steer_command(self.packer, apply_steer, apply_steer_req, frame))











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
