from can import *

import time

def same_data(ldata, rdata, printData = False):
  if len(ldata) != len(rdata):
    return False
  for (l, r) in zip(ldata, rdata) :
    if l == r:
      if printData:
        print str(l) + " == " + str(r)
    else: 
      return False
  return True

def assert_valid_canid(can_id):
  if not 1 <= can_id <= 127:
    print("id must be between 1 and 127")
    sys.exit(-1)

def assert_same_data(ldata, rdata, print_data = False):
  if not same_data(ldata, rdata, print_data):
    print("the data is not the same")
    sys.exit(-1)

def sleepms(ms):
  time.sleep(ms/1000)
