#!/usr/bin/env python

import sys
import rospy
from sawyer_utils.sawyer_head_control import SawyerHeadControl

import intera_interface
import intera_external_devices
import numpy as np
from intera_interface import CHECK_VERSION

def print_bindings():
    print("key bindings: ")
    print("  Esc: Quit")
    print("  ?: Help")
    print("  a: Pan Left 1 degree")
    print("  d: Pan Right 1 degree")
    print("  q: Pan Left 5 degree")
    print("  e: Pan Right 5 degree")
    print("  r: Zero Head Position")

def map_keyboard(head):
    done = False
    while not done and not rospy.is_shutdown():
        c = intera_external_devices.getch()
        if c:
            #catch Esc or ctrl-c
            if c in ['\x1b', '\x03']:
                done = True
                rospy.signal_shutdown("Exiting.")
            elif c == 'a':
                head.move_head(np.pi/180.0)
            elif c == 'd':
                head.move_head(-np.pi/180.0)
            elif c == 'q':
                head.move_head(5*np.pi/180.0)
            elif c == 'e':
                head.move_head(-5*np.pi/180.0)
            elif c == 'r':
                head.set_head(0.0)
            else:
                print_bindings()

def main():
    print_bindings()    
    rospy.init_node("sawyer_head_control")
    head = SawyerHeadControl()
    rs = intera_interface.RobotEnable(CHECK_VERSION)
    rs.enable()
    map_keyboard(head)

if __name__ == '__main__':
    main()
