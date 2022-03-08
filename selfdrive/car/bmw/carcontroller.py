from cereal import car
from common.numpy_fast import clip, interp
from selfdrive.car import apply_toyota_steer_torque_limits, create_gas_command, make_can_msg
from selfdrive.car.bmw.bmwcan import create_steer_command, create_accel_command
from selfdrive.car.bmw.values import CAR, SteerActuatorParams, SteerLimitParams
from opendbc.can.packer import CANPacker
from selfdrive.config import Conversions as CV

VisualAlert = car.CarControl.HUDControl.VisualAlert
import time

def calc_steering_torque_hold(angle, vEgo):
  
  return SteerActuatorParams.CENTERING_COEFF * angle #interpolate between zero hold torque and linear region starting point at a given vehicle speed

SAMPLING_FREQ = 100 #Hz

# Accel limits
ACCEL_HYST_GAP = 0.02  # don't change accel command for small oscilalitons within this value
ACCEL_MAX = 4  # cruise control rapid clicking
ACCEL_SLOW = 3 # cruise control hold up
DECEL_SLOW = -2   # cruise control decrease speed slowly
DECEL_MIN = -6  # cruise control hold down
ACCEL_SCALE = max(ACCEL_MAX, -DECEL_MIN)


# Steer angle limits
ANGLE_MAX_BP = [5., 15., 30]  #m/s
ANGLE_MAX = [200., 20., 10.] #deg
ANGLE_RATE_BP = [0., 5., 15.]      # m/s
ANGLE_RATE_WINDUP = [500., 80., 15.]     #deg/s windup rate limit
ANGLE_RATE_UNWIND = [500., 350., 40.]  #deg/s unwind rate limit

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
    self.last_target_angle_lim = 0
    self.accel_steady = 0.
    self.alert_active = False
    self.last_standstill = False
    self.standstill_req = False
    self.angle_control = False
    self.last_frame_cruise_cmd_sent = 0
    self.last_type_cruise_cmd_sent = 0
    self.cruise_speed_prev = 0
    self.steer_angle_enabled = False
    self.last_fault_frame = -200

    #don't rename
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



    # detect check driver pressing as well
    # check whether speed changed in the direction that was commanded - covers OP and driver's commands
    if (self.cruise_speed_prev - CS.out.cruiseState.speed) != 0:
      self.last_type_cruise_cmd_sent = self.cruise_speed_prev - CS.out.cruiseState.speed

      # it should only take one frame for car to update cruise speed. If cruise speed changed after two frames,
      #it was probably driver pressing stalks. In that case update the timestamp too
      if frame - self.last_frame_cruise_cmd_sent > 1: #ignore if cruiseState.speed changed shortly after sending command
        self.last_frame_cruise_cmd_sent = frame

    frames_since_cruise_sent = frame - self.last_frame_cruise_cmd_sent

    # hysteresis
    speed_diff_req = requestedSpeed - CS.out.cruiseState.speed
    CC_STEP = 1 * CV.KPH_TO_MS  # metric car settings
    speed_margin_thresh = 0.1 * CV.KPH_TO_MS
    hysteresis_timeout = 20 #200ms
    # hysteresis, since cruiseState.speed changes in steps and
    if self.last_type_cruise_cmd_sent > 0 and frames_since_cruise_sent < hysteresis_timeout: #maybe instead just look at vTargetFuture
      speed_diff_err_up =   speed_margin_thresh
      speed_diff_err_dn =  -CC_STEP
    elif self.last_type_cruise_cmd_sent < 0 and frames_since_cruise_sent < hysteresis_timeout:
      speed_diff_err_up =  CC_STEP
      speed_diff_err_dn = -speed_margin_thresh
    else:
      speed_diff_err_up = CC_STEP / 2 + speed_margin_thresh
      speed_diff_err_dn = -CC_STEP / 2

    cruise_tick = 20 # default rate
    if (control.actuators.brake > 0.2 or control.actuators.gas > 0.2) and abs(speed_diff_req)>1:
      cruise_tick = 10   #-1 no delay - emulate held stalk (keep sending messages at 100Hz) to make bmw brake or accelerate hard
    # elif round(abs(speed_diff_req)) == 1:
    #   self.last_frame_cruise_cmd_sent = frame #reset counter
    #   cruise_tick = 35 #slow period


    if CC_cancel_cmd and frames_since_cruise_sent > cruise_tick:
      can_sends.append(create_accel_command(self.packer, "cancel", self.cruise_bus, frame))
      self.last_frame_cruise_cmd_sent = frame
      self.last_type_cruise_cmd_sent = 0
      print("cancel")
    elif speed_diff_req > speed_diff_err_up and control.enabled and frames_since_cruise_sent > cruise_tick:
      can_sends.append(create_accel_command(self.packer, "plus1", self.cruise_bus, frame))
      self.last_frame_cruise_cmd_sent = frame
      self.last_type_cruise_cmd_sent = +1
    elif speed_diff_req < speed_diff_err_dn and control.enabled and frames_since_cruise_sent > cruise_tick and not CS.out.gasPressed:
      can_sends.append(create_accel_command(self.packer, "minus1", self.cruise_bus, frame))
      self.last_frame_cruise_cmd_sent = frame
      self.last_type_cruise_cmd_sent = -1

    self.cruise_speed_prev = CS.out.cruiseState.speed


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
      angle_lim = interp(CS.out.vEgo, ANGLE_MAX_BP, ANGLE_MAX)
      target_angle_lim = clip(control.actuators.steerAngle, -angle_lim, angle_lim)
      
      # windup slower #todo implement real (speed) rate limiter
      if (self.last_target_angle_lim * target_angle_lim) > 0. and abs(target_angle_lim) > abs(self.last_target_angle_lim): #todo revise last_angle
        angle_rate_max = interp(CS.out.vEgo, ANGLE_RATE_BP, ANGLE_RATE_WINDUP) 
      else:
        angle_rate_max = interp(CS.out.vEgo, ANGLE_RATE_BP, ANGLE_RATE_UNWIND)
      
      # steer angle - don't allow too large delta
      MAX_SEC_BEHIND = 1 #seconds behind target. Target deltas behind more than 1s will be rejected by bmw_safety
      target_angle_lim = clip(target_angle_lim, self.last_target_angle_lim - angle_rate_max*MAX_SEC_BEHIND, self.last_target_angle_lim + angle_rate_max*MAX_SEC_BEHIND)
      
      target_angle_delta =  target_angle_lim - CS.out.steeringAngle
      angle_deltastep_max = angle_rate_max / SAMPLING_FREQ
      angle_desired_rate = clip(target_angle_delta, -angle_deltastep_max, angle_deltastep_max) #apply max allowed rate such that the target is not overshot within a sample
      
      self.steer_rate_limited = target_angle_delta != angle_desired_rate #todo #desired rate only drives stepper (inertial) holding torque in this iteration. Rate is limited independently in Trinamic controller
      
      # steer torque
      I_steering = 0.005 #estimated moment of inertia (inertia of a ring = I=mR^2 = 2kg * .15^2 = 0.045kgm2)
      inertia_tq = I_steering * ((angle_desired_rate * SAMPLING_FREQ - CS.out.steeringRate ) * SAMPLING_FREQ) * CV.DEG_TO_RAD  #kg*m^2 * rad/s^2 = N*m (torque)
      
      # add feed-forward and inertia compensation
      steer_tq = calc_steering_torque_hold(target_angle_lim, CS.out.vEgo) + inertia_tq

      # explicitly clip torque before sending on CAN
      steer_tq = clip(steer_tq, -SteerActuatorParams.MAX_STEERING_TQ, SteerActuatorParams.MAX_STEERING_TQ)
      
      can_sends.append(create_steer_command(int(True), target_angle_delta, steer_tq, frame))
      # *** control msgs ***
      if (frame % 10) == 0: #slow print
        print("SteerAngleErr {0} Inertia  {1} Brake {2}, SpeedDiff {3}".format(control.actuators.steerAngle - CS.out.steeringAngle,
                                                                 inertia_tq,
                                                                 control.actuators.brake, speed_diff_req))
    else:
      target_angle_lim = CS.out.steeringAngle
      can_sends.append(create_steer_command(int(False), 0., 0., frame)) 
      
      # if (frame % 100) == 0: #slow print when disabled
      #   print("SteerAngle {0} SteerSpeed {1}".format(CS.out.steeringAngle,
                                                                #  CS.out.steeringRate))


    self.last_steer = apply_hold_torque
    self.last_target_angle_lim = target_angle_lim
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
