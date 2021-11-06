#define ENABLED_ACTUATOR 0 // GMLAN_HIGH 12V-> thru NPN -> ENN_pin=0V -> Trinamic drive stage enabled
#define DISABLED_ACTUATOR 1 // GMLAN_LOW 0V-> thru NPN -> ENN_pin=5V -> Trinamic drive stage disabled

const AddrBus BMW_TX_MSGS[] = {{404, 0}, {404, 1}, {252, 2}};
int SAMPLING_FREQ = 100; //Hz

bool bmw_fmax_limit_check(float val, const float MAX_VAL, const float MIN_VAL) {
  return (val > MAX_VAL) || (val < MIN_VAL);
}
 
// 2m/s are added to be less restrictive
float BMW_ANGLE_MARGIN = 0.5;

const struct lookup_t BMW_LOOKUP_MAX_ANGLE = {
    {5., 15., 30.},     // m/s
    {200., 20., 10.}};  // deg
    

const struct lookup_t BMW_ANGLE_RATE_WINDUP = { // deg/s windup rate limit
    {0., 5., 15.},      // m/s
    {500., 80., 15.}};  // deg/s

const struct lookup_t BMW_ANGLE_RATE_UNWIND = { // deg/s unwind rate limit
    {0., 5., 15.},      // m/s
    {500., 350., 40.}}; // deg/s

const struct lookup_t BMW_MAX_CURR = {
    {2., 29., 38.},
    {410., 92., 36.}};

const uint32_t BMW_RT_INTERVAL = 250000; // 250ms between real time checks

// state of angle limits
float bmw_desired_angle_last = 0; // last desired steer angle
float bmw_rt_angle_last = 0.; // last real time angle

float angle_rate_up;
float angle_rate_down;
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
    } else {
      cruise_engaged = 0;
    }
    if (!cruise_engaged) {
      controls_allowed = false;
    } 
    if (cruise_engaged && !cruise_engaged_last) {
      controls_allowed = true;
    }
    cruise_engaged_last = cruise_engaged;
  }

  if (addr == 404){ //disable on cruise stalk cancel
    if ((GET_BYTE(to_push, 2) & 0x90) != 0x0){
      controls_allowed = false;
    }
  }
  if (addr == 466) { //TransmissionDataDisplay  
    if ((GET_BYTE(to_push, 0) & 0xF) == ((GET_BYTE(to_push, 0) >> 4) ^ 0xF)) { //check agains shift lever compliment signal 
      lever_position = GET_BYTE(to_push, 0) & 0xF; //compliment match
    } else {
      lever_position = -1; //invalid
    }
    // if not in Drive
    if (lever_position != 8 ){
      controls_allowed = false;
    }
  }

  // exit controls on brake press
  if (addr == 168) {
    // any of two bits at position 61 & 62
    if ((GET_BYTE(to_push, 7)  & 0x60) != 0) {
      controls_allowed = false;
    }
  }
  //get vehicle speed
  if (addr == 416) {
    bmw_speed = ((((GET_BYTE(to_push, 1) & 0xF) << 8) | GET_BYTE(to_push, 0)) * 0.103 * 0.277778); //raw to km/h to m/s
    angle_rate_up = interpolate(BMW_ANGLE_RATE_WINDUP, bmw_speed) + BMW_ANGLE_MARGIN;   // deg/1s
    angle_rate_down = interpolate(BMW_ANGLE_RATE_UNWIND, bmw_speed) + BMW_ANGLE_MARGIN; // deg/1s
    bmw_max_angle = interpolate(BMW_LOOKUP_MAX_ANGLE, bmw_speed) + BMW_ANGLE_MARGIN;
  }


  // todo exit controls on actuator error

  //get latest steering wheel angle rate
  if (addr == 196) {
    float meas_angle = to_signed((GET_BYTE(to_push, 1) << 8) | GET_BYTE(to_push, 0), 16) * 0.0439453125; //deg
    float angle_rate = to_signed((GET_BYTE(to_push, 4) << 8) | GET_BYTE(to_push, 3), 16) * 0.0439453125; //deg/s

    if(bmw_fmax_limit_check(meas_angle, bmw_max_angle, -bmw_max_angle)){
      // We should not be able to STEER under these conditions
      controls_allowed = false;
      puts("Too big angle \n");
    }
    
    if (bmw_fmax_limit_check((bmw_rt_angle_last >= 0.) ? angle_rate : -angle_rate, angle_rate_up, -angle_rate_down)) { //should be sensitive for jerks to the outside
      controls_allowed = false;         // todo  ^ handle zero crossing a bit smarter
      puts("To fast angle rate \n");
    }
    
    if ((controls_allowed && !bmw_controls_allowed_last)) {
      bmw_rt_angle_last = meas_angle;
    }
  }
  
  if (controls_allowed == true){
    set_gmlan_digital_output(ENABLED_ACTUATOR);
  #ifdef ALLOW_DEBUG
  } else if((bmw_speed == 0.) && ((lever_position == 0) || (lever_position == 1))) {//activate motor with bmw safety when external tool transmitting and shiflt lever is off or in Park
    //do nothing unless calibration tool transmitting:
    if(addr == 252) {
      puts("Debug enable gmlan\n");
      set_gmlan_digital_output(ENABLED_ACTUATOR);
      reset_gmlan_switch_timeout();
    }
  #endif
  } else {
    set_gmlan_digital_output(DISABLED_ACTUATOR);
  }
  controls_allowed = true;  //TODO: safety was disabled for debugging!!!!!, re-enable when done
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
  int bus = GET_BUS(to_send);

   if (!msg_allowed(addr, bus, BMW_TX_MSGS, sizeof(BMW_TX_MSGS) / sizeof(BMW_TX_MSGS[0]))) {
    tx = 0;
  }

  // do not transmit CAN message if steering angle too high
  if (addr == 252) {
    if((GET_BYTE(to_send, 0) & 0xFF) == 4){ //TMCL MVP comannd
      if ((GET_BYTE(to_send, 1) & 0xFF) == 1){ //relative movement type only
        float angle_ratio = 25. / 12. * (26. + (103./121.)) * 256. / 1.8;
        float angle_delta_req = ((float)(int32_t)((GET_BYTE(to_send, 3) << 24) | (GET_BYTE(to_send, 4) << 16) | (GET_BYTE(to_send, 5) << 8) | GET_BYTE(to_send, 6))) / angle_ratio; //deg/10ms
        float desired_angle = bmw_rt_angle_last + angle_delta_req; //measured + requested delta

        if (controls_allowed == true) {
          bool violation = false;
          //check for max angles
          violation |= bmw_fmax_limit_check(desired_angle, bmw_max_angle, -bmw_max_angle);
          puts("bmw_fmax_limit_check desired_angle\n");
          //angle is rate limited in carcontrols so it shouldn't exceed max delta
          violation |= bmw_fmax_limit_check(((bmw_desired_angle_last >= 0.) ? angle_delta_req : -angle_delta_req), angle_rate_up, -angle_rate_down);
          puts("bmw_fmax_limit_check bmw_desired_angle_last \n");

          if (violation) {
            tx = 0;
            controls_allowed = false;
          }
          bmw_desired_angle_last = desired_angle;
        }
      }else{ //incorect MVP command, don't send
        tx = 0;
      }
    }
  }
  if(controls_allowed == false){
    tx = 0;
  }
  
  if (tx != 0) { //todo this rather should not exist
    reset_gmlan_switch_timeout();
  }
  if ((addr == 0xFC) && controls_allowed) {
    reset_gmlan_switch_timeout();
  }

  return tx;
}

static void bmw_init(int16_t param) {
  UNUSED(param);
  controls_allowed = false;
  bmw_speed = 0;
  lever_position = -1;

  #ifdef ALLOW_DEBUG
    puts("BMW safety init\n");
  #endif
  gmlan_switch_init(1); //init the gmlan switch with 1s timeout enabled
  set_gmlan_digital_output(DISABLED_ACTUATOR);
}

const safety_hooks bmw_hooks = {
  .init = bmw_init,
  .rx = bmw_rx_hook,
  .tx = bmw_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = default_fwd_hook,
};
