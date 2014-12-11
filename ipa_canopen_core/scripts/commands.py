#!/usr/bin/env python

def set_velocity(p, can_id, velocity):
  command_sdo_key = SDOKey(0x1023, 1)
  send_segmented_data(p, can_id, command_sdo_key, 'JV=' + str(velocity))
  
def set_abs_position(p, can_id, position):
  command_sdo_key = SDOKey(0x1023, 1)
  send_segmented_data(p, can_id, command_sdo_key, 'PA=' + str(position))
  
def begin_motion(p, can_id):
  command_sdo_key = SDOKey(0x1023, 1)
  send_segmented_data(p, can_id, command_sdo_key, 'BG')
  
def stop_motion(p, can_id):
  command_sdo_key = SDOKey(0x1023, 1)
  send_segmented_data(p, can_id, command_sdo_key, 'ST')
  
def enable_motor(p, can_id):
  command_sdo_key = SDOKey(0x1023, 1)
  send_segmented_data(p, can_id, command_sdo_key, 'MO=1')
  
def disable_motor(p, can_id):
  command_sdo_key = SDOKey(0x1023, 1)
  send_segmented_data(p, can_id, command_sdo_key, 'MO=0')

