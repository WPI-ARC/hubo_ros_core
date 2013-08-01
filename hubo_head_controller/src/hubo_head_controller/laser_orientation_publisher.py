#!/usr/bin/env python

#   Calder Phillips-Grafflin - WPI/ARC Lab

import rospy
import math
import tf
from tf.transformations import *
from geometry_msgs.msg import *
from transformation_helper import *
from sensor_msgs.msg import *

class LaserOrientationPublisher:

    def __init__(self, laser_frame, root_frame, laser_joint, head_variant, rate):
        self.laser_frame = laser_frame
        self.root_frame = root_frame
        self.rate = rate
        self.laser_joint = laser_joint
        self.last_laser_joint = 0.0
        self.base_transform = Transform()
        if (head_variant == "prebeta"):
            self.base_transform.translation.x = 0.0
            self.base_transform.translation.y = -0.035
            self.base_transform.translation.z = 0.04
        else:
            self.base_transform.translation.x = 0.10
            self.base_transform.translation.y = -0.035
            self.base_transform.translation.z = 0.04
        self.laser_offset_transform = Transform()
        self.laser_offset_transform.translation.x = 0.0
        self.laser_offset_transform.translation.y = 0.0
        self.laser_offset_transform.translation.z = 0.08
        self.laser_offset_transform.rotation.x = 0.0
        self.laser_offset_transform.rotation.y = 0.0
        self.laser_offset_transform.rotation.z = 0.0
        self.laser_offset_transform.rotation.w = 1.0
        self.broadcaster = tf.TransformBroadcaster()
        self.joint_sub = rospy.Subscriber("joint_states", JointState, self.orientation_cb)
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            [x, y, z, w] = quaternion_about_axis(self.last_laser_joint, (0,1,0))
            self.base_transform.rotation.x = x
            self.base_transform.rotation.y = y
            self.base_transform.rotation.z = z
            self.base_transform.rotation.w = w
            laser_transform = ComposeTransforms(self.base_transform, self.laser_offset_transform)
            [lt, lr] = ComponentsFromTransform(laser_transform)
            self.broadcaster.sendTransform(lt, lr, rospy.Time.now(), self.laser_frame, self.root_frame)
            rate.sleep()

    def orientation_cb(self, msg):
        for index in range(len(msg.name)):
            if (self.laser_joint == msg.name[index]):
                self.last_laser_joint = msg.position[index]
                break

if __name__ == "__main__":
    rospy.init_node("generic_tf_broadcaster")
    rospy.loginfo("Starting the laser orientation broadcaster...")
    #Get the parameters from the server
    root_frame = rospy.get_param("~root_frame", "Body_NK2")
    laser_frame = rospy.get_param("~laser_frame", "laser_sensor_frame")
    head_variant = rospy.get_param("~head_variant", "prebeta")
    laser_joint = rospy.get_param("~laser_joint", "NK3")
    rate = rospy.get_param("~rate", 100.0)
    LaserOrientationPublisher(laser_frame, root_frame, laser_joint, head_variant, rate)
