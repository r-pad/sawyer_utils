#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import tf.transformations as trans

base_standoff = 0.2

def initTableScene(scene, table_shape = [0.7, 1.6], table_height = -0.17):
    scene.remove_world_object()

    time_stamp = rospy.get_rostime()
    
    table_center = [base_standoff + table_shape[0]/2.0, 0.0, table_height]    
    table_pose = PoseStamped()
    table_pose.header.frame_id = 'base'
    table_pose.header.stamp = time_stamp
    table_pose.pose.orientation.w = 1.0
    table_pose.pose.position.x = table_center[0]
    table_pose.pose.position.y = table_center[1]
    table_pose.pose.position.z = table_center[2]
    table_size =  tuple(table_shape) + (0.0, )

    wall_height = 2.0
    wall_b_angle = -0.40970294454245623
    
    wall_b_size = (0.0, 1.05, wall_height)
    wall_bl_pose = PoseStamped()
    wall_bl_pose.header.frame_id = 'base'
    wall_bl_pose.header.stamp = time_stamp
    wall_bl_quat = trans.quaternion_about_axis(wall_b_angle, (0,0,1))
    wall_bl_pose.pose.orientation.x = wall_bl_quat[0]
    wall_bl_pose.pose.orientation.y = wall_bl_quat[1]
    wall_bl_pose.pose.orientation.z = wall_bl_quat[2]
    wall_bl_pose.pose.orientation.w = wall_bl_quat[3]
    wall_bl_pose.pose.position.x = 0.0
    wall_bl_pose.pose.position.y = 0.5
    wall_bl_pose.pose.position.z = wall_height/2.0 + table_center[2]
    wall_br_pose = PoseStamped()
    wall_br_pose.header.frame_id = 'base'
    wall_br_pose.header.stamp = time_stamp
    wall_br_quat = trans.quaternion_about_axis(-wall_b_angle, (0,0,1))
    wall_br_pose.pose.orientation.x = wall_br_quat[0]
    wall_br_pose.pose.orientation.y = wall_br_quat[1]
    wall_br_pose.pose.orientation.z = wall_br_quat[2]
    wall_br_pose.pose.orientation.w = wall_br_quat[3]
    wall_br_pose.pose.position.x = 0.0
    wall_br_pose.pose.position.y = -0.5
    wall_br_pose.pose.position.z = wall_height/2.0 + table_center[2]
    
    wall_s_size = (table_size[0], 0.0, wall_height)
    wall_sl_pose = PoseStamped()
    wall_sl_pose.header.frame_id = 'base'
    wall_sl_pose.header.stamp = time_stamp
    wall_sl_pose.pose.orientation.w = 1
    wall_sl_pose.pose.position.x = table_center[0]
    wall_sl_pose.pose.position.y = table_size[1]/2.0
    wall_sl_pose.pose.position.z = wall_height/2.0 + table_center[2]
    wall_sr_pose = PoseStamped()
    wall_sr_pose.header.frame_id = 'base'
    wall_sr_pose.header.stamp = time_stamp
    wall_sr_pose.pose.orientation.w = 1
    wall_sr_pose.pose.position.x = table_center[0]
    wall_sr_pose.pose.position.y = -table_size[1]/2.0
    wall_sr_pose.pose.position.z = wall_height/2.0 + table_center[2]

    wall_f_size = (0.0, table_size[1], wall_height)
    wall_f_pose = PoseStamped()
    wall_f_pose.header.frame_id = 'base'
    wall_f_pose.header.stamp = time_stamp
    wall_f_pose.pose.orientation.w = 1
    wall_f_pose.pose.position.x = table_center[0] + table_size[0]/2.0
    wall_f_pose.pose.position.y = 0.0
    wall_f_pose.pose.position.z = wall_height/2.0 + table_center[2]

    rospy.sleep(0.2)
    scene.add_box('table', table_pose, table_size)
    scene.add_box('wall_f', wall_f_pose, wall_f_size)
    scene.add_box('wall_bl', wall_bl_pose, wall_b_size)
    scene.add_box('wall_br', wall_br_pose, wall_b_size)    
    scene.add_box('wall_sl', wall_sl_pose, wall_s_size)
    scene.add_box('wall_sr', wall_sr_pose, wall_s_size)
    rospy.sleep(0.1)

gripper_standoff = 0.02

def addGripperObject(scene, object_size = None, object_center = None, object_quat = None, 
                     object_name = 'object', clear_gripper=True):
    time_stamp = rospy.get_rostime()
    if(object_size is None):
        object_size = [.05, .09, .09]
    if(object_center is None):
        object_center = [0,0, object_size[2]/2.0 + gripper_standoff]
    if(object_quat is None):
        object_quat = [0,0,0,1]
        
    obj_pose = PoseStamped()
    obj_pose.header.frame_id = 'right_gripper'
    obj_pose.header.stamp = time_stamp

    obj_pose.pose.orientation.x = object_quat[0]
    obj_pose.pose.orientation.y = object_quat[1]
    obj_pose.pose.orientation.z = object_quat[2]
    obj_pose.pose.orientation.w = object_quat[3]
    
    obj_pose.pose.position.x = object_center[0]
    obj_pose.pose.position.y = object_center[1]
    obj_pose.pose.position.z = object_center[2]
    
    if(clear_gripper):
        clearGripper(scene)
    scene.attach_box('right_gripper', object_name, obj_pose, object_size)
    rospy.sleep(0.1)
    
def clearGripper(scene):
    objects = scene.get_attached_objects()
    scene.remove_attached_object('right_gripper')
    for obj in objects.keys():
        scene.remove_world_object(obj)
        