#define ENABLED_ACTUATOR 0 // GMLAN_HIGH 12V-> thru NPN -> ENN_pin=0V -> Trinamic drive stage enabled
#define DISABLED_ACTUATOR 1 // GMLAN_LOW 0V-> thru NPN -> ENN_pin=5V -> Trinamic drive stage disabled

bool bmw_fmax_limit_check(float val, const float MAX_VAL, const float MIN_VAL) {
  return (val > MAX_VAL) || (val < MIN_VAL);
}

// 2m/s are added to be less restrictive

const struct lookup_t BMW_LOOKUP_MAX_ANGLE = {
    {5., 15., 30.},
    {200., 20., 10.}};
    
const struct lookup_t BMW_LOOKUP_ANGLE_RATE_UP = {
    {0., 5., 15.},
    {500., 80., 15.}};  // deg/s

const struct lookup_t BMW_LOOKUP_ANGLE_RATE_DOWN = {
    {0., 5., 15.},
    {500., 350., 40.}}; // deg/s


const struct lookup_t BMW_LOOKUP_MAX_CURR = {
    {2., 29., 38.},
    {410., 92., 36.}};

const uint32_t BMW_RT_INTERVAL = 250000; // 250ms between real time checks

// state of angle limits
float bmw_desired_angle_last = 0; // last desired steer angle
float bmw_rt_angle_last = 0.; // last real time angle

float delta_angle_up;
float delta_angle_down;
float bmw_max_angle;
int bmw_controls_allowed_last = 0;

int lever_position = -1; //0 is when no ignition, so -1 unset
float bmw_speed = 0;

void set_gmlan_digital_output(int to_set);
void reset_gmlan_switch_timeout(void);
void gmlan_switch_init(int timeout_enable);

int cruise_engaged_last = 0;

static int bmw_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {
  int addr = GET_ADDR(to_push);

  if ((addr == 0x193) || (addr == 0x200)) { //handles both vehicle options VO544 and Vo540
    int cruise_engaged = 0;
    if (addr == 0x193) { //dynamic cruise control
      cruise_engaged = ((GET_BYTE(to_push, 5) >> 3) & 1);
    } else if (addr == 0x200) { //normal cruise control option
      cruise_engaged = ((GET_BYTE(to_push, 1) >> 5) & 1);
    }
    if (!cruise_engaged) {
      controls_allowed = 0;
    } 
    if (cruise_engaged && !cruise_engaged_last) {
      controls_allowed = 1;
    }
    cruise_engaged_last = cruise_engaged;
  }

  if (addr == 0x194){ //disable on cruise stalk cancel
    if ((GET_BYTE(to_push, 2) >> 4) & 1) {
      controls_allowed = 0;
      }
  }
  if (addr == 0x1D2) { //TransmissionDataDisplay  
    if ((GET_BYTE(to_push, 0) & 0xF) == ((GET_BYTE(to_push, 0) >> 4) ^ 0xF)) { //check agains shift lever compliment signal 
      lever_position = GET_BYTE(to_push, 0) & 0xF; //compliment match
    } else {
      lever_position = -1; //invalid
    }
    // if not in Drive
    if (lever_position != 8 ){
      controls_allowed = 0;
    }
  }
  // exit controls on brake press
  if (addr == 0xA8) {
    // any of two bits at position 61 & 62
    if ((GET_BYTE(to_push, 7)  & 0x60) != 0) {
      controls_allowed = 0;
    }
  }
  //get vehicle speed
  if (addr == 0x1a0) {
    bmw_speed = (((GET_BYTE(to_push, 1) & 0xF) << 8) + GET_BYTE(to_push, 0) * 0.103 * 0.277778); //raw to km/h to m/s
    delta_angle_up = interpolate(BMW_LOOKUP_ANGLE_RATE_UP, bmw_speed) + 1.;
    delta_angle_down = interpolate(BMW_LOOKUP_ANGLE_RATE_DOWN, bmw_speed) + 1.;
    bmw_max_angle = interpolate(BMW_LOOKUP_MAX_ANGLE, bmw_speed) + 1.;
  }


  // exit controls on actuator error
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
  //get latest steering wheel angle rate
  if (addr == 0xC4) {
    float bmw_meas_angle = ((int) ((GET_BYTE(to_push, 0) << 8) + GET_BYTE(to_push, 1))) * 0.0428317;
    float angle_rate = ((int) ((GET_BYTE(to_push, 3) << 8) + GET_BYTE(to_push, 4))) * 0.0428317;
    // puts("\nAngle measured: "); puth(angle_rate);


    if(bmw_fmax_limit_check(bmw_meas_angle, -bmw_max_angle, bmw_max_angle)){
      // We should not be able to STEER under these conditions
      controls_allowed = 0;
      puts("MAX! \n");
    }
    
    if (bmw_fmax_limit_check(bmw_rt_angle_last > 0 ? angle_rate : -angle_rate, delta_angle_up, delta_angle_down)) { //should be sensitive for jerks to the outside
      controls_allowed = 0;
      puts("RATE! \n");
    }
    
    if ((controls_allowed && !bmw_controls_allowed_last)) {
      bmw_rt_angle_last = bmw_meas_angle;
    }
  }
  
  if (controls_allowed){
    set_gmlan_digital_output(ENABLED_ACTUATOR);
  #ifdef ALLOW_DEBUG
  } else if((bmw_speed == 0) && ((lever_position == 0) || (lever_position == 1))) {//activate motor with bmw safety when external tool transmitting and shiflt lever is off or in Park
    //do nothing unless calibration tool transmitting:
    if(addr == 0xFC) {
      puts("Debug enable gmlan\n");
      set_gmlan_digital_output(ENABLED_ACTUATOR);
      reset_gmlan_switch_timeout();
    }
  #endif
  } else {
    set_gmlan_digital_output(DISABLED_ACTUATOR);
  }
  
  bmw_controls_allowed_last = controls_allowed;
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
  // set_gmlan_digital_output(ENABLED_ACTUATOR);
  // do not transmit CAN message if steering angle too high
  if ((addr == 0xFC) && false ) {
    if((GET_BYTE(to_send, 0) & 0xFF) == 04){ //TMCL MVP comannd
      if ((GET_BYTE(to_send, 1) & 0xFF) == 01){ //relative movement type only
        float angle_delta_req = ((GET_BYTE(to_send, 3) << 24) + (GET_BYTE(to_send, 4) << 16) + (GET_BYTE(to_send, 5) << 8) + GET_BYTE(to_send, 6)) / (1.8 * 256 * 27 * 25/12);
        float desired_angle = bmw_rt_angle_last + (angle_delta_req); //measured + requested delta
        bool violation = 0;

        if (controls_allowed) {
          //check for max angles
          violation |= bmw_fmax_limit_check(desired_angle, bmw_max_angle, -bmw_max_angle);
          //angle is rate limited in carcontrols so it shouldn't exceed max delta
          violation |= bmw_fmax_limit_check(bmw_desired_angle_last > 0 ? angle_delta_req : -angle_delta_req, delta_angle_up, delta_angle_down);
          
          if (violation) {
            tx = 0;
          }
          bmw_desired_angle_last = desired_angle;
        } else {
          tx = 0;
        }
      }else{ //incorect MVP command, don't send
        tx = 0;
      }
    }
  }
  if (tx) {
    reset_gmlan_switch_timeout();
  }


  if ((addr == 0xFC) && controls_allowed) {
      reset_gmlan_switch_timeout();
  }

  return tx;
}

static void bmw_init(int16_t param) {
  UNUSED(param);
  controls_allowed = 0;
  bmw_speed = 0;
  lever_position = -1;

  puts("BMW safety init\n");
  gmlan_switch_init(1); //init the gmlan switch with 1s timeout enabled
  set_gmlan_digital_output(DISABLED_ACTUATOR);
  puts("gmlan out\n");
}

const safety_hooks bmw_hooks = {
  .init = bmw_init,
  .rx = bmw_rx_hook,
  .tx = bmw_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = default_fwd_hook,
};
