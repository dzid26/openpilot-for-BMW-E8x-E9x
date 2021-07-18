#!/usr/bin/env python3

import os
import sys
import time

sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), ".."))
from panda import Panda
import _thread

def set_safety(id):
  p = Panda()
  p.set_safety_mode(id)
  start_heartbeat_thread(p)
  input('Press key to stop')


def start_heartbeat_thread(p):
  def heartbeat_thread(p):
    while True:
      try:
        p.send_heartbeat()
        p.set_power_save(False)
        time.sleep(1)
      except:
        break
  _thread.start_new_thread(heartbeat_thread, (p,))

if __name__ == "__main__":
  safety_id = int(os.getenv("SAFETY", 0))  #run in console ./set_safety.py 1 for Honda
  set_safety(12)
