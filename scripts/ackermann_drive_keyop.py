#!/usr/bin/env python

'''
ackermann_drive_keyop.py:
    A ros keyboard teleoperation script for ackermann steering based robots
'''

__author__ = 'George Kouros'
__license__ = 'GPLv3'
__maintainer__ = 'George Kouros'
__email__ = 'gkourosg@yahoo.gr'

import roslib
import rospy
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float64
import sys, select, termios, tty
import thread
from numpy import clip

control_keys = {
    'up'    : '\x41',
    'down'  : '\x42',
    'right' : '\x43',
    'left'  : '\x44',
    'space' : '\x20',
    'tab'   : '\x09'}

key_bindings = {
    '\x41' : ( 1.0 , 0.0),
    '\x42' : (-1.0 , 0.0),
    '\x43' : ( 0.0 ,-1.0),
    '\x44' : ( 0.0 , 1.0),
    '\x20' : ( 0.0 , 0.0),
    '\x09' : ( 0.0 , 0.0)}


class AckermannDriveKeyop:

    def __init__(self, max_speed=0.22, max_steering_angle=0.7):
        self.speed_range = [-float(max_speed), float(max_speed)]
        self.steering_angle_range = [-float(max_steering_angle),
                                     float(max_steering_angle)]
        for key in key_bindings:
            key_bindings[key] = \
                    (key_bindings[key][0] * float(max_speed) / 5,
                     key_bindings[key][1] * float(max_steering_angle) / 5)

        self.speed = 0
        self.steering_angle = 0
        self.motors_pub = rospy.Publisher(
            'ackermann_cmd', AckermannDrive, queue_size=1)
        rospy.Timer(rospy.Duration(1.0/5.0), self.pub_callback, oneshot=False)
        self.key_loop()

    def pub_callback(self, event):
        ackermann_cmd_msg = AckermannDrive()
        ackermann_cmd_msg.speed = self.speed
        ackermann_cmd_msg.steering_angle = self.steering_angle
        self.motors_pub.publish(ackermann_cmd_msg)

    def print_state(self):
        sys.stderr.write('\x1b[2J\x1b[H')
        rospy.loginfo('\x1b[1M\r*********************************************')
        rospy.loginfo('\x1b[1M\rUse arrows to change speed and steering angle')
        rospy.loginfo('\x1b[1M\rUse space to brake and tab to align wheels')
        rospy.loginfo('\x1b[1M\rPress <ctrl-c> or <q> to exit')
        rospy.loginfo('\x1b[1M\r*********************************************')
        rospy.loginfo('\x1b[1M\r\033[32;1mSpeed: %0.2f m/s, '
                      'Steer Angle: %0.1f rad\033[0m',
                      self.speed, self.steering_angle)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key


    def key_loop(self):
        self.settings = termios.tcgetattr(sys.stdin)
        while 1:
            key = self.get_key()
            if key in key_bindings.keys():
                if key == control_keys['space']:
                    self.speed = 0.0
                elif key == control_keys['tab']:
                    self.steering_angle = 0.0
                else:
                    self.speed = self.speed + key_bindings[key][0]
                    self.steering_angle = \
                            self.steering_angle + key_bindings[key][1]
                    self.speed = clip(
                        self.speed, self.speed_range[0], self.speed_range[1])
                    self.steering_angle = clip(
                        self.steering_angle,
                        self.steering_angle_range[0],
                        self.steering_angle_range[1])
                self.print_state()
            elif key == '\x03' or key == '\x71':  # ctr-c or q
                break
            else:
                continue
        self.finalize()

    def finalize(self):
        rospy.loginfo('Halting motors, aligning wheels and exiting...')
        self.settings = termios.tcgetattr(sys.stdin)
        ackermann_cmd_msg = AckermannDrive()
        ackermann_cmd_msg.speed = 0
        ackermann_cmd_msg.steering_angle = 0
        self.motors_pub.publish(ackermann_cmd_msg)
        sys.exit()

if __name__ == '__main__':
    sys.stderr.write('\x1b[2J\x1b[H')
    rospy.init_node('ackermann_drive_keyop_node')
    rospy.loginfo('Ackermann Drive Keyboard Teleoperation Node Initialized')
    args = sys.argv[1:]
    if len(args) == 2:
        rospy.loginfo('Setting motors max speed to %s and steering angle to %s',
                      args[0], args[1])
        keyop = AckermannDriveKeyop(args[0], args[1])
    else:
        rospy.loginfo('Using default motors max speed(0.5m/s) and steering '
                      'angle(0.8rad)')
        keyop = AckermannDriveKeyop()
