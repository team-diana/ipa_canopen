#!/usr/bin/env python
from can import *
from canutils import *
import sys
import time
import itertools

def send_segmented_sdo(port, can_id, data, toggle, more_segments):
    obj_id = can_id + 0x600 

    data_len = len(data)
    if data_len > 7:
      print("Error: data was > 7 bytes long")
      sys.exit(-1)

    toggle_bit = 0b10000 if toggle else 0
    more_segments_bit = 0b1 if more_segments else 0
    size_bits = (7-data_len) << 1
    first_byte = [ 0x01 + toggle_bit + more_segments_bit + size_bits]
    packet_data = itertools.chain(first_byte, data)
    port.send_msg(obj_id, list(packet_data), 0)

def get_sdo_key_from_data(data):
    return SDOKey(data[1] + (data[2] << 8), data[3])

def chunks(l, n):
    """ Yield successive n-sized chunks from l. """
    for i in xrange(0, len(l), n):
            yield l[i:i+n]

def send_segmented_data(port, can_id, sdo_key, data):
    port.init_send_segmented_sdo(can_id, sdo_key)
    sleepms(1000)
    res_data = port.recv_msg().data
    received_sdo_key = get_sdo_key_from_data(res_data)
    if received_sdo_key != sdo_key:
      print "ERROR: unexpected response while init segmented sdo send"
    splitted_data = list(chunks(data, 7))

    toggle = False
    chunk_num = 1

    for chunk in splitted_data:
      more_segments = True
      if len(splitted_data) == chunk_num:
        more_segments = False 
      send_segmented_sdo(port, can_id, chunk, toggle, more_segments)
      sleepms(1000)
      res_data = port.recv_msg().data
      expected_first_byte = 0x20
      toggle_bit = 0b10000 if toggle else 0
      if res_data[0] != (0x20 + toggle_bit):
        print("ERROR bad server response ")
      toggle = not toggle
      chunk_num = chunk_num + 1
      
def str_to_data_g(string):
    for c in string: 
      yield ord(c)

def str_to_data(string):
    return list(str_to_data_g(string))

def init_os_mode(p, can_id):
    sdo_key = SDOKey(0x1024, 0)
    p.send_sdo(can_id, sdo_key, 0, True)
    sleepms(3000)
    resp = p.recv_msg()
    expectedResData = [0x60, 0x24, 0x10, 0, 0, 0, 0, 0]
    if not same_data(expectedResData, resp.data):
        raise Exception("unable to set evaluate immediately mode")
    sleepms(100)
    p.clean_buffers()
    sdo_key = SDOKey(0x1024, 0)
    p.send_sdo(can_id, sdo_key, 0, True)
    sleepms(3000)

def start_shell(can_id):
    p = CanPort(0, 0)
    p.open()
    p.config_port(mode.BIT11, 0, 0x7EE, baud_rate.CAN_BAUD_500K) # TODO: check all parameters
    sleepms(100)
    p.clean_buffers()

    p.reset_communication()
    sleepms(2000)
    #p.start_remote_node(can_id)
    #sleepms(4000)

    p.clean_buffers()

    sdo_key = SDOKey(0x1024, 0)
    p.send_sdo(can_id, sdo_key, 0, True)

    sleepms(3000)
    resp = p.recv_msg()
    expectedResData = [0x60, 0x24, 0x10, 0, 0, 0, 0, 0]
    if not same_data(expectedResData, resp.data):
        raise Exception("unable to set evaluate immediately mode")

    sleepms(100)
    p.clean_buffers()
    command_sdo_key = SDOKey(0x1023, 1)

    
    stop = False
    print ("enter 'q' or 'exit' to finish the program")
    while not stop:
      s = raw_input("-->:")
      if s == "q" or s == "exit":
        stop = True
      else:
        send_segmented_data(p, can_id, command_sdo_key, str_to_data(s))
        sleepms(400)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("usage: set-canid CAN_ID")
        sys.exit(-1)
    can_id = int(sys.argv[1])
    start_shell(can_id)
