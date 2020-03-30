// board enforces
//   in-state
//      accel set/resume
//   out-state
//      cancel button
//      regen paddle
//      accel rising edge
//      brake rising edge
//      brake > 0mph
//
#define ENABLED_ACTUATOR GMLAN_HIGH // GMLAN_HIGH 12V-> thru NPN -> ENN_pin=0V -> Trinamic drive stage enabled
#define DISABLED_ACTUATOR GMLAN_LOW // GMLAN_LOW 0V-> thru NPN -> ENN_pin=5V -> Trinamic drive stage disabled

bool bmw_fmax_limit_check(float val, const float MAX_VAL, const float MIN_VAL) {
  return (val > MAX_VAL) || (val < MIN_VAL);
}

// 2m/s are added to be less restrictive
const struct lookup_t BMW_LOOKUP_ANGLE_RATE_UP = {
    {2., 7., 17.},
    {5., .8, .25}};

const struct lookup_t BMW_LOOKUP_ANGLE_RATE_DOWN = {
    {2., 7., 17.},
    {5., 3.5, .8}};

const struct lookup_t BMW_LOOKUP_MAX_ANGLE = {
    {2., 29., 38.},
    {410., 92., 36.}};

const uint32_t BMW_RT_INTERVAL = 250000; // 250ms between real time checks

// state of angle limits
float bmw_desired_angle_last = 0; // last desired steer angle
float bmw_rt_angle_last = 0.; // last real time angle
float bmw_ts_angle_last = 0;

int bmw_controls_allowed_last = 0;

int bmw_brake_prev = 0;
int bmw_gas_prev = 0;
int bmw_speed = 0;
//int eac_status = 0;

void set_gmlan_digital_output(int to_set);
void reset_gmlan_switch_timeout(void);
void gmlan_switch_init(int timeout_enable);

int cruise_engaged_last = 0;

static int bmw_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {
    //set_gmlan_digital_output(ENABLED_ACTUATOR);

  int addr = GET_ADDR(to_push);

  if ((addr == 0x193) || (addr == 0x200)) { //handles both vehicle options VO544 and Vo540
    int cruise_engaged = 0;
    if (addr == 0x193) { //dynamic cruise control
      cruise_engaged = ((GET_BYTE(to_push, 5) >> 3) & 1);
    } else if (addr == 0x200) { //normal cruise control option
      cruise_engaged = ((GET_BYTE(to_push, 1) >> 5) & 1);
    }
    if (cruise_engaged && !cruise_engaged_last) {
      controls_allowed = 1;
    }
    if (!cruise_engaged) {
      controls_allowed = 0;
    }
    cruise_engaged_last = cruise_engaged;
  }



  if (addr == 0x45) {
    // 6 bits starting at position 0
    // int lever_position = GET_BYTE(to_push, 0) & 0x3F;
    // if (lever_position == 2) { // pull forward
    //   // activate openpilot
    //   controls_allowed = 1;
    // }
    // if (lever_position == 1) { // push towards the back
    //   // deactivate openpilot
    //   controls_allowed = 0;
    // }
  }

  // exit controls on brake press
  if (addr == 0x168) {
    // any of two bits at position 61&62
    if ((GET_BYTE(to_push, 8)  & 0x60) != 0) {
      // disable break cancel by commenting line below
      controls_allowed = 0;
    }
    //get vehicle speed in m/s. Bmw gives MPH
    bmw_speed = (((((GET_BYTE(to_push, 3) & 0xF) << 8) + GET_BYTE(to_push, 2)) * 0.05) - 25) * 1.609 / 3.6;
    if (bmw_speed < 0) {
      bmw_speed = 0;
    }
  }

  // exit controls on EPAS error
  // EPAS_sysStatus::EPAS_eacStatus 0x370
//  if (addr == 0x370) {
//    // if EPAS_eacStatus is not 1 or 2, disable control
////    eac_status = (GET_BYTE(to_push, 6) >> 5) & 0x7;
////    // For human steering override we must not disable controls when eac_status == 0
////    // Additional safety: we could only allow eac_status == 0 when we have human steering allowed
////    if (controls_allowed && (eac_status != 0) && (eac_status != 1) && (eac_status != 2)) {
////      controls_allowed = 0;
////      //puts("EPAS error! \n");
////    }
//  }
  //get latest steering wheel angle
  if (addr == 0x00E) {
    float angle_meas_now = (int)(((((GET_BYTE(to_push, 0) & 0x3F) << 8) + GET_BYTE(to_push, 1)) * 0.1) - 819.2);
    uint32_t ts = TIM2->CNT;
    uint32_t ts_elapsed = get_ts_elapsed(ts, bmw_ts_angle_last);

    // *** angle real time check
    // add 1 to not false trigger the violation and multiply by 25 since the check is done every 250 ms and steer angle is updated at     100Hz
    float rt_delta_angle_up = (interpolate(BMW_LOOKUP_ANGLE_RATE_UP, bmw_speed) * 25.) + 1.;
    float rt_delta_angle_down = (interpolate(BMW_LOOKUP_ANGLE_RATE_DOWN, bmw_speed) * 25.) + 1.;
    float highest_rt_angle = bmw_rt_angle_last + ((bmw_rt_angle_last > 0.) ? rt_delta_angle_up : rt_delta_angle_down);
    float lowest_rt_angle = bmw_rt_angle_last - ((bmw_rt_angle_last > 0.) ? rt_delta_angle_down : rt_delta_angle_up);

    if ((ts_elapsed > BMW_RT_INTERVAL) || (controls_allowed && !bmw_controls_allowed_last)) {
      bmw_rt_angle_last = angle_meas_now;
      bmw_ts_angle_last = ts;
    }

    // check for violation;
    if (bmw_fmax_limit_check(angle_meas_now, highest_rt_angle, lowest_rt_angle)) {
      // We should not be able to STEER under these conditions
      // Other sending is fine (to allow human override)
      controls_allowed = 0;
      //puts("WARN: RT Angle - No steer allowed! \n");
    } else {
      controls_allowed = 1;
    }

    bmw_controls_allowed_last = controls_allowed;
  }
  return 1;
}

// all commands: gas/regen, friction brake and steering
// if controls_allowed and no pedals pressed
//     allow all commands up to limit
// else
//     block all commands that produce actuation

static int bmw_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {

  int tx = 1;
  int addr = GET_ADDR(to_send);
  //set_gmlan_digital_output(ENABLED_ACTUATOR);
  // do not transmit CAN message if steering angle too high
  // DAS_steeringControl::DAS_steeringAngleRequest
  if (addr == 0x488) {
    float angle_raw = ((GET_BYTE(to_send, 0) & 0x7F) << 8) + GET_BYTE(to_send, 1);
    float desired_angle = (angle_raw * 0.1) - 1638.35;
    bool violation = 0;
    int st_enabled = GET_BYTE(to_send, 2) & 0x40;

    if (st_enabled == 0) {
      //steering is not enabled, do not check angles and do send
      bmw_desired_angle_last = desired_angle;
    } else if (controls_allowed) {
      // add 1 to not false trigger the violation
      float delta_angle_up = interpolate(BMW_LOOKUP_ANGLE_RATE_UP, bmw_speed) + 1.;
      float delta_angle_down = interpolate(BMW_LOOKUP_ANGLE_RATE_DOWN, bmw_speed) + 1.;
      float highest_desired_angle = bmw_desired_angle_last + ((bmw_desired_angle_last > 0.) ? delta_angle_up : delta_angle_down);
      float lowest_desired_angle = bmw_desired_angle_last - ((bmw_desired_angle_last > 0.) ? delta_angle_down : delta_angle_up);
      float BMW_MAX_ANGLE = interpolate(BMW_LOOKUP_MAX_ANGLE, bmw_speed) + 1.;

      //check for max angles
      violation |= bmw_fmax_limit_check(desired_angle, BMW_MAX_ANGLE, -BMW_MAX_ANGLE);

      //check for angle delta changes
      violation |= bmw_fmax_limit_check(desired_angle, highest_desired_angle, lowest_desired_angle);

      if (violation) {
        controls_allowed = 0;
        tx = 0;
      }
      bmw_desired_angle_last = desired_angle;
    } else {
      tx = 0;
    }
  }
  return tx;
}

static void bmw_init(int16_t param) {
  UNUSED(param);
  controls_allowed = 0;
  puts("BMW safety init\n");
  gmlan_switch_init(0); //init the gmlan switch with 1s timeout enabled
  //set_gmlan_digital_output(ENABLED_ACTUATOR);
}

const safety_hooks bmw_hooks = {
  .init = bmw_init,
  .rx = bmw_rx_hook,
  .tx = bmw_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = default_fwd_hook,
};
