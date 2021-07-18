#!/usr/bin/env python3
import unittest
import numpy as np
from panda import Panda
from panda.tests.safety import libpandasafety_py
from panda.tests.safety.common import StdTest, make_msg

ANGLE_DELTA_BP = [0., 5., 15.]
ANGLE_DELTA_V = [5., .8, .15]     # windup limit
ANGLE_DELTA_VU = [5., 3.5, 0.4]   # unwind limit

ENABLED_ACTUATOR = 0 # GMLAN_HIGH 12V-> thru NPN -> ENN_pin=0V -> Trinamic drive stage enabled
DISABLED_ACTUATOR = 1 # GMLAN_LOW 0V-> thru NPN -> ENN_pin=5V -> Trinamic drive stage disabled

TX_MSGS = [[0x194, 0],[0x194, 1], [0xFC, 2]]

def twos_comp(val, bits):
  if val >= 0:
    return val
  else:
    return (2**bits) + val

def sign(a):
  if a > 0:
    return 1
  else:
    return -1


class TestBmwSafety(unittest.TestCase):
  @classmethod
  def setUp(cls):
    cls.safety = libpandasafety_py.libpandasafety
    cls.safety.set_safety_hooks(Panda.SAFETY_BMW, 0)
    cls.safety.init_tests_bmw()

  def _angle_meas_msg(self, angle, angle_rate):
    to_send = make_msg(0, 0xc4)
    angle = int(angle / 0.0439453125)
    t = twos_comp(angle, 16) # signed
    to_send[0].RDLR = (t & 0xFFFF)


    return to_send

  def _set_prev_angle(self, t):
    t = int(t * -100)
    self.safety.set_bmw_desired_angle_last(t)

  def _angle_meas_msg_array(self, angle):
    for i in range(6):
      self.safety.safety_rx_hook(self._angle_meas_msg(angle))

  def _actuator_angle_cmd_msg(self, angle, state):

    #Trinamic options:
    address = 252
    to_send = make_msg(0, address)
    cmd = 4 #MVP
    cmdtype = 1  #Relative
    bank = 0
    val = twos_comp(int(angle * 25 / 12  * 27 * 256 / 1.8 ), 32) # signed
    checksum = (cmd + cmdtype + bank + val) % 255

    to_send[0].RDLR = ((cmd & 0xFF) | ((cmdtype & 0xFF) << 8) | ((bank & 0xFF) << 16) | (val & 0xFF000000 ))
    to_send[0].RDHR = (val & 0x00FFFFFF) | ((checksum & 0xFF) << 24)

    return to_send


  def _actuator_curr_cmd_msg(self, steer_current, state):

    #Trinamic options:
    address = 253
    to_send = make_msg(0, address)
    cmd = 5  #SAP
    cmdtype = 6  #_APs.MaxCurrent
    bank = 0
    MAX_CURRENT = 1.2
    val = int(steer_current / MAX_CURRENT * 255 ) & 0xFF
    checksum = (cmd + cmdtype + bank + val) % 255

    to_send[0].RDLR = ((cmd & 0xFF) | ((cmdtype & 0xFF) << 8) | ((bank & 0xFF) << 16) | (val & 0xFF000000 ))
    to_send[0].RDHR = (val & 0x00FFFFFF) | ((checksum & 0xFF) << 24)

    return to_send

  def _speed_msg(self, speed):
    to_send = make_msg(0, 0x1a0)
    speed = int(speed / 0.103)
    to_send[0].RDLR = (speed & 0xFFF)

    return to_send

  def _brake_msg(self, brake):
    to_send = make_msg(1, 0xa8)
    to_send[0].RDHR = ((brake & 0x1) << 29)

    return to_send

  def _acc_button_cmd(self, buttons): #todo: read creuisesate
    to_send = make_msg(2, 404)
    to_send[0].RDLR = (buttons << 20)

    return to_send

  def test_spam_can_buses(self):
    StdTest.test_spam_can_buses(self, TX_MSGS)

  def test_angle_cmd_when_enabled(self):
    # when controls are allowed, angle cmd rate limit is enforced
    speeds = [ 5., 10., 15., 50., 100] #kph
    angles = [-300, -100, -10, 0, 10, 100, 300] #deg
    for a in angles:
      for s in speeds:
        max_delta_up = np.interp(s, ANGLE_DELTA_BP, ANGLE_DELTA_V)
        max_delta_down = np.interp(s, ANGLE_DELTA_BP, ANGLE_DELTA_VU)

        # first test against false positives
        self._angle_meas_msg_array(a)
        self.safety.safety_rx_hook(self._speed_msg(s))

        self._set_prev_angle(a)
        self.safety.set_controls_allowed(1)

        # Stay within limits
        # Up
        self.assertEqual(True, self.safety.safety_tx_hook(self._actuator_angle_cmd_msg(a + sign(a) * max_delta_up, 1)))
        self.assertTrue(self.safety.get_controls_allowed())

        # Don't change
        self.assertEqual(True, self.safety.safety_tx_hook(self._actuator_angle_cmd_msg(a, 1)))
        self.assertTrue(self.safety.get_controls_allowed())

        # Down
        self.assertEqual(True, self.safety.safety_tx_hook(self._actuator_angle_cmd_msg(a - sign(a) * max_delta_down, 1)))
        self.assertTrue(self.safety.get_controls_allowed())

        # Inject too high rates
        # Up
        self.assertEqual(False, self.safety.safety_tx_hook(self._actuator_angle_cmd_msg(a + sign(a) * (max_delta_up + 1), 1)))
        self.assertFalse(self.safety.get_controls_allowed())

        # Don't change
        self.safety.set_controls_allowed(1)
        self._set_prev_angle(a)
        self.assertTrue(self.safety.get_controls_allowed())
        self.assertEqual(True, self.safety.safety_tx_hook(self._actuator_angle_cmd_msg(a, 1)))
        self.assertTrue(self.safety.get_controls_allowed())

        # Down
        self.assertEqual(False, self.safety.safety_tx_hook(self._actuator_angle_cmd_msg(a - sign(a) * (max_delta_down + 1), 1)))
        self.assertFalse(self.safety.get_controls_allowed())

        # Check desired steer should be the same as steer angle when controls are off
        self.safety.set_controls_allowed(0)
        self.assertEqual(True, self.safety.safety_tx_hook(self._actuator_angle_cmd_msg(a, 0)))

  def test_angle_cmd_when_disabled(self):
    self.safety.set_controls_allowed(0)

    self._set_prev_angle(0)
    self.assertFalse(self.safety.safety_tx_hook(self._actuator_angle_cmd_msg(0, 1)))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_brake_disengage(self):
    self.safety.set_controls_allowed(1)
    self.safety.set_gmlan_digital_output(ENABLED_ACTUATOR)
    self.safety.safety_rx_hook(self._brake_msg(1))
    self.safety.safety_tx_hook(self._brake_msg(1))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertEqual(self.safety.get_gmlan_digital_output(), DISABLED_ACTUATOR)

  def test_acc_buttons(self):
    self.safety.set_controls_allowed(1)
    self.safety.safety_tx_hook(self._acc_button_cmd(0x2)) # Cancel button
    self.assertTrue(self.safety.get_controls_allowed())
    self.safety.safety_tx_hook(self._acc_button_cmd(0x20)) # No button pressed
    self.assertFalse(self.safety.get_controls_allowed())

  def test_fwd_hook(self):

    buss = list(range(0x0, 0x3))
    msgs = list(range(0x1, 0x800))

    blocked_msgs = [(2, 0x169), (2, 0x2b1), (2, 0x4cc), (0, 0x280)]
    for b in buss:
      for m in msgs:
        if b == 0:
          fwd_bus = 2
        elif b == 1:
          fwd_bus = -1
        elif b == 2:
          fwd_bus = 0

        if (b, m) in blocked_msgs:
          fwd_bus = -1

        # assume len 8
        self.assertEqual(fwd_bus, self.safety.safety_fwd_hook(b, make_msg(b, m, 8)))

if __name__ == "__main__":
  unittest.main()
