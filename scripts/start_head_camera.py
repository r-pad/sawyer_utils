#!/usr/bin/env python

import rospy
import intera_interface

rospy.init_node("start_head_camera")
camera = intera_interface.Cameras()
camera_name = "head_camera"
camera.start_streaming(camera_name)