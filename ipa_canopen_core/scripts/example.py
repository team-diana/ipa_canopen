#!/usr/bin/env python

from can import *
import time
import sys
import random

def assert_same_data(ldata, rdata, printData = False):
  if len(ldata) != len(rdata):
    print "error, different size"
    sys.exit(-1)

  for (l, r) in zip(ldata, rdata) :
    if l == r:
      if printData:
        print str(l) + " == " + str(r)
    else: 
      print "error, wrong value!"
      sys.exit(-1)

def main():
  port0 = CanPort(0, 0)
  port1 = CanPort(0, 1)

  port0.enable_debug(True)
  port1.enable_debug(True)

  print "opening ports"
  port0.open()
  port1.open()

  port0.clean_buffers()
  port1.clean_buffers()

  print "setup ports"
  port0.config_port(mode.BIT11, 0, 0x7ff, baud_rate.CAN_BAUD_125K)
  port1.config_port(mode.BIT11, 0, 0x7ff, baud_rate.CAN_BAUD_125K)

  print "sending msg"
  data =  [0, 1, 2, 3, 251, 252, 253, 254]
  port0.send_msg(0, data, 0)

  time.sleep(0.050)

  print "receiving msg"
  msg = port1.recv_msg()

  assert_same_data(data, msg.data, True)

  # do random test
  port0.enable_debug(False)
  port1.enable_debug(False)
  print "doing random test, will take a while..."
  for i in range(1, 10000):
    data = [random.randint(1, 254) for x in range(1, 8)] 
    port0.send_msg(0, data, 0) 
    time.sleep(0.050)
    msg = port1.recv_msg()
    assert_same_data(data, msg.data, False)



if __name__ == "__main__" :
  print "starting..."
  main()

