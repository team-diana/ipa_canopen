#!/usr/bin/env python

from can import *
from canutils import *
import sys

def set_id(can_id):
  port0 = CanPort(0, 0)
  port0.enable_debug(True)

  port0.open()
  port0.clean_buffers()

  port0.config_port(mode.BIT11, 0, 0x7ff, baud_rate.CAN_BAUD_500K)

  print("setting config msg")
  port0.send_msg(0x7e5, [4, 1, 0, 0,  0, 0, 0, 0], 0)
  print("waiting for restart...")
  sleepms(5000)
  print("sending configuration...")
  port0.send_msg(0x7e5, [17, can_id, 0, 0,  0, 0, 0, 0], 0)
  sleepms(1000)
  res = port0.recv_msg()
  expected_data = [0x11, 0, 0, 0,  0, 0, 0, 0]
  if same_data(expected_data, res.data, False):
    print("OK: id was set.") 
  else:
    print("ERROR: id was not set.") 
    sys.exit(-1)

if __name__ == "__main__" :
  if len(sys.argv) != 2:
    print("usage: set-canid CAN_ID")
    sys.exit(-1)
  can_id = int(sys.argv[1])
  print("setting can id to " + str(can_id))
  assert_valid_canid(can_id)
  set_id(can_id)

