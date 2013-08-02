#!/usr/bin/python

#################################################
#                                               #
#   Calder Phillips-Grafflin - WPI/ARC Lab      #
#                                               #
#   Laser pointing controller for DRCHubo       #
#                                               #
#   This provides one interface:                #
#                                               #
#   1.  LaserScanAction                         #
#                                               #
#################################################

import rospy
import math
from std_msgs.msg import *
from sensor_msgs.msg import *
import actionlib
from geometry_msgs.msg import *
import hubo_robot_msgs.msg as hrms
import hubo_sensor_msgs.msg as hsms

def test_head():
    head_client = actionlib.SimpleActionClient("/drchubo_point_head", hrms.PointHeadAction)
    head_client.wait_for_server()
    head_goal = hrms.PointHeadGoal()
    head_goal.pointing_frame = "/Body_NK2"
    head_goal.pointing_axis.x = 1.0
    head_goal.target.header.frame_id = "/Body_Torso"
    head_goal.target.point.x = -5.0
    head_goal.target.point.y = 0.0
    head_goal.target.point.z = 0.0
    head_goal.min_duration = rospy.Duration(5.0)
    print "Sending PointHeadGoal"
    head_client.send_goal(head_goal)
    head_client.wait_for_result()
    print "Action finished"


def test_lidar():
    laser_client = actionlib.SimpleActionClient("/drchubo_laser_scan", hsms.LaserScanAction)
    laser_client.wait_for_server()
    lidar_goal = hsms.LaserScanGoal()
    lidar_goal.min_angle = 0.25
    lidar_goal.max_angle = -0.25
    lidar_goal.tilt_rate = 0.25
    print "Sending LaserScanGoal"
    laser_client.send_goal(lidar_goal)
    laser_client.wait_for_result()
    print "Action finished"

if __name__ == '__main__':
    rospy.init_node("head_tester")
    test_lidar()
    test_head()
