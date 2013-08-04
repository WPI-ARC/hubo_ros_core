#!/usr/bin/env python

#   Calder Phillips-Grafflin - WPI/ARC Lab

import rospy
import math
import tf
from tf.transformations import *
from sensor_msgs.msg import *

class BodyOrientationPublisher:

    def __init__(self, target_frame, root_frame, imu_topic, rate):
        self.target_frame = target_frame
        self.root_frame = root_frame
        self.rate = rate
        self.tx = 0.0
        self.ty = 0.0
        self.tz = 0.0
        self.rx = 0.0
        self.ry = 0.0
        self.rz = 0.0
        self.rw = 1.0
        self.broadcaster = tf.TransformBroadcaster()
        self.imu_sub = rospy.Subscriber(imu_topic, Imu, self.orientation_cb)
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.broadcaster.sendTransform((self.tx, self.ty, self.tz), (self.rx, self.ry, self.rz, self.rw), rospy.Time.now(), self.target_frame, self.root_frame)
            rate.sleep()

    def orientation_cb(self, msg):
        ax = -msg.linear_acceleration.x
        ay = -msg.linear_acceleration.y
        # So these values aren't actually acceleration
        # instead, they are estimated roll and pitch
        rq = quaternion_about_axis(ax, (1,0,0))
        pq = quaternion_about_axis(ay, (0,1,0))
        [x, y, z, w] = self.NormalizeQuaternion(self.ComposeQuaternions(rq, pq))
        self.rx = x
        self.ry = y
        self.rz = z
        self.rw = w

    def ComposeQuaternions(self, q1, q2):
        x = q1[3]*q2[0] + q2[3]*q1[0] + q1[1]*q2[2] - q2[1]*q1[2]
        y = q1[3]*q2[1] + q2[3]*q1[1] + q2[0]*q1[2] - q1[0]*q2[2]
        z = q1[3]*q2[2] + q2[3]*q1[2] + q1[0]*q2[1] - q2[0]*q1[1]
        w = q1[3]*q2[3] - q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2]
        return [x,y,z,w]

    def NormalizeQuaternion(self, q_raw):
        magnitude = math.sqrt(q_raw[0]**2 + q_raw[1]**2 + q_raw[2]**2 + q_raw[3]**2)
        x = q_raw[0] / magnitude
        y = q_raw[1] / magnitude
        z = q_raw[2] / magnitude
        w = q_raw[3] / magnitude
        return [x,y,z,w]

if __name__ == "__main__":
    rospy.init_node("body_orientation_publisher")
    rospy.loginfo("Starting the body orientation broadcaster...")
    #Get the parameters from the server
    root_frame = rospy.get_param("~root_frame", "sensor_imu_frame")
    target_frame = rospy.get_param("~target_frame", "world_orientation_frame")
    imu_topic = rospy.get_param("~imu_topic", "")
    rate = rospy.get_param("~rate", 100.0)
    BodyOrientationPublisher(target_frame, root_frame, imu_topic, rate)
