#!/usr/bin/env python

from os_shell import *
from commands import *

class Motor():
    def __init__(self, can_port, can_id):
        self.can_id = can_id
        self.can_port = can_port
        init_os_mode(self.can_port, self.can_id)

    def enable(self):
        enable_motor(self.can_port, self.can_id)

    def start(self): 
        begin_motion(self.can_port, self.can_id)

    def disable(self):
        self.stop()
        disable_motor(self.can_port, self.can_id)

    def set_vel(self, vel):
        set_velocity(self.can_port, self.can_id, vel)        
