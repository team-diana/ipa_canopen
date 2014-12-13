#!/usr/bin/env python

import rospy

from geometry_msgs import Twist
from motor import Motor
from math import pi

class Controller():
    def __init__(self):
        self.node = rospy.init_node("wheel_controller")
        self.command_sub = rospy.Subscriber("name", Twist, self.command_callback)
        rospy.on_shutdown(self.on_shutdown)
        self.can_port = CanPort(0, 0)
        self.init_can_port()
        can_ids = [11, 12, 13, 14]
        self.motors = [Motor(self.can_port, can_id) for can_id in can_ids]
        self.wheel_separation = 0.030
        self.wheel_radius = 0.01

    def wheel_speed(self, rpm):
        return self.wheel_radius * rpm * 2 * pi / 60 

    def init_can_port(self):
        p = self.can_port
        p.open()
        p.config_port(mode.BIT11, 0, 0x7EE, baud_rate.CAN_BAUD_500K) # TODO: check all parameters
        sleepms(100)
        p.clean_buffers()
        p.reset_communication()
        sleepms(2000)
        p.clean_buffers()

    def init_motors():
        [m.enable() for m in motors]

    def command_callback(self, twist_msg):
        velocities = evaluate_velocities(twist_msg.linear.x, twist_msg.angular.z)
        for (motor, vel) in zip(self.motors, velocities):
            motor.set_vel(vel)

    def evaluate_velocities(vr, va):
        right_v = (vr + va * self.wheel_separation_ / 2.0);
        left_v = (vr - va * self.wheel_separation_ / 2.0);

        return (right_v, right_v, left_v, left_v) 


    def run(self):
        rospy.spin()

    def on_shutdown(self):
        [m.disable() for m in motors]


if __name__ == "__main__":
    controller = Controller()
    controller.run()
