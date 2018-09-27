#! /usr/bin/env python

import numpy as np

import sys
import rospy
import moveit_commander
import intera_interface

class SawyerJointControl(object):
    def __init__(self, scene = None):
        moveit_commander.roscpp_initialize(sys.argv)
        if(scene is None):
            self.scene = moveit_commander.PlanningSceneInterface()
        else:
            self.scene = scene
            
        self.group = moveit_commander.MoveGroupCommander("right_arm")
        self.limb = intera_interface.Limb('right')
        self.joints = self.limb.joint_names()
        
    def move_joint(self, joint_idx, delta):
        joint_name = self.joints[joint_idx]
        current_position = self.limb.joint_angle(joint_name)
        joint_command = {joint_name: current_position + delta}
        self.limb.set_joint_positions(joint_command)
        
    def set_joint(self, joint_idx, theta):
        joint_name = self.joints[joint_idx]
        joint_command = {joint_name: theta}
        self.limb.set_joint_positions(joint_command)

    def get_joint(self, joint_idx):
        joint_values = self.limb.joint_angles().values()[::-1]
        return joint_values[joint_idx]
    
    def get_config(self):
        joint_values = self.limb.joint_angles().values()[::-1]
        return joint_values

    def get_endpoint_pose(self, as_array = False):
        current_pose = self.limb.endpoint_pose()
        current_position = current_pose['position']
        current_orientation = current_pose['orientation']
        if as_array:
            return np.array([current_position.x, current_position.y, current_position.z]), \
                np.array([current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w])
        else:
            return current_position, current_orientation

    def goto_joint(self, joint_idx, theta):
        self.group.clear_pose_targets()
        joint_values = self.limb.joint_angles().values()[::-1]
        joint_values[joint_idx] = theta
        self.group.set_joint_value_target(joint_values)
        self.group.plan()
        self.group.go(wait=True)
        
    def goto_config(self, joint_values):
        self.group.clear_pose_targets()
        self.group.set_joint_value_target(joint_values)
        self.group.plan()
        self.group.go(wait=True)

    def goto_pose(self, end_effector_pose):
        self.group.clear_pose_targets()
        self.group.set_pose_target(end_effector_pose)
        self.group.plan()
        self.group.go(wait=True)
        
    def is_done(self, linear_vel_threshold=5e-3, angular_vel_threshold=1e-2):
        linear_vel = np.array(self.limb.endpoint_velocity()['linear'])
        angular_vel = np.array(self.limb.endpoint_velocity()['angular'])
        return np.linalg.norm(linear_vel) < linear_vel_threshold and \
            np.linalg.norm(angular_vel) < angular_vel_threshold
            
    def wait_until_done(self, max_wait_time = 5.0,
                        linear_vel_threshold=5e-3, 
                        angular_vel_threshold=1e-2):
        start_t = rospy.get_time()
        while(not self.is_done(linear_vel_threshold, angular_vel_threshold) and not rospy.is_shutdown() and \
              rospy.get_time() - start_t < max_wait_time):
            rospy.sleep(0.1)            
        
        return self.is_done(linear_vel_threshold, angular_vel_threshold)