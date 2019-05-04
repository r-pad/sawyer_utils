# -*- coding: utf-8 -*-
"""
Created on Sun Apr 29 14:05:28 2018

@author: bokorn
"""
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import intera_interface
import numpy as np
from sawyer_utils.sawyer_joint_control import SawyerJointControl
import sawyer_utils.moveit_scene_utils as scene_utils


try:
    rospy.init_node("test_joint_control")
    scene = moveit_commander.PlanningSceneInterface()
    
    sawyer = SawyerJointControl(scene)
    scene_utils.initTableScene(scene, table_shape = [1.0, 1.6])
    #scene_utils.addGripperObject(scene)
    
    import IPython; IPython.embed()
except rospy.ROSInterruptException:
    pass
