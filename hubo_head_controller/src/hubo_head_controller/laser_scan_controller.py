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
import tf
from tf.transformations import *
from transformation_helper import *
import actionlib
from geometry_msgs.msg import *
import hubo_robot_msgs.msg as hrms
import hubo_sensor_msgs.msg as hsms
import dynamixel_msgs.msg as dmms

class LaserScanController:

    def __init__(self, tilt_controller_prefix, zero_tilt_position, max_tilt_position, min_tilt_position, laser_topic, target_angular_rate, error_threshold):
        self.zero_tilt_position = zero_tilt_position
        self.max_tilt_position = max_tilt_position
        self.min_tilt_position = min_tilt_position
        self.target_angular_rate = target_angular_rate
        self.error_threshold = error_threshold
        self.running = True
        self.active = False
        self.laser_scans = []
        self.last_tilt_state = None
        rospy.loginfo("Configuring LaserScanController...")
        rospy.loginfo("tilt_controller_prefix = " + tilt_controller_prefix)
        rospy.loginfo("laser_topic = " + laser_topic)
        self.tilt_subscriber = rospy.Subscriber(tilt_controller_prefix + "/state", dmms.JointState, self.tilt_state_cb)
        self.laser_scan_subscriber = rospy.Subscriber(laser_topic, LaserScan, self.laser_scan_cb)
        rospy.loginfo("Loaded subscribers to laser states")
        retry_rate = rospy.Rate(1.0)
        while (self.last_tilt_state == None and not rospy.is_shutdown()):
            rospy.loginfo("Waiting for state from laser tilt controller...")
            retry_rate.sleep()
        rospy.loginfo("...laser tilt controller ready")
        rospy.loginfo("Loading laser controller command publisher...")
        self.tilt_command_pub = rospy.Publisher(tilt_controller_prefix + "/command", Float64)
        rospy.loginfo("...Loaded command publisher")
        rospy.loginfo("...LaserScanController setup finished")

    def tilt_state_cb(self, msg):
        self.last_tilt_state = msg

    def laser_scan_cb(self, msg):
        if (self.active):
            self.laser_scans.append([self.last_laser_state.current_pos, msg])

    def SelfTest(self):
        rospy.loginfo("Self-testing LaserScan Controller")
        zero_traj = self.BuildTrajectory(self.last_tilt_state.current_pos, self.zero_tilt_position, self.target_angular_rate)
        result = self.RunTrajectory(zero_traj)
        if (result):
            rospy.loginfo("Moved to zero position")
        else:
            rospy.logerr("Unable to zero")
            return result
        max_traj = self.BuildTrajectory(self.last_tilt_state.current_pos, self.max_tilt_position, self.target_angular_rate)
        result = self.RunTrajectory(max_traj)
        if (result):
            rospy.loginfo("Moved to max position")
        else:
            rospy.logerr("Unable to reach max position")
            return result
        min_traj = self.BuildTrajectory(self.last_tilt_state.current_pos, self.min_tilt_position, self.target_angular_rate)
        result = self.RunTrajectory(min_traj)
        if (result):
            rospy.loginfo("Moved to min position")
        else:
            rospy.logerr("Unable to reach min position")
            return result
        zero_traj = self.BuildTrajectory(self.last_tilt_state.current_pos, self.zero_tilt_position, self.target_angular_rate)
        result = self.RunTrajectory(zero_traj)
        if (result):
            rospy.loginfo("Self-test procedure complete")
        else:
            rospy.logerr("Unable to complete self-test")
        return result

    def RunActionServer(self):
        self.server = actionlib.SimpleActionServer('drchubo_laser_scan', hsms.LaserScanAction, self.execute_laser_scan, False)
        self.server.start()
        safety_rate = rospy.Rate(10.0)
        while not rospy.is_shutdown() and self.running:
            safety_rate.sleep()
        del(self.server)
        return False

    def execute_laser_scan(self, laser_scan_goal):
        max_safe = self.CheckSafetyBounds(laser_scan_goal.max_angle)
        min_safe = self.CheckSafetyBounds(laser_scan_goal.min_angle)
        if ((not max_safe) or (not min_safe)):
            rospy.logerr("Aborting LaserScanAction Goal due to safety violations")
            self.server.set_aborted()
        else:
            prep_traj = self.BuildTrajectory(self.last_tilt_state.current_pos, laser_scan_goal.min_angle, self.target_angular_rate)
            scan_traj = self.BuildTrajectory(laser_scan_goal.min_angle, laser_scan_goal.max_angle, laser_scan_goal.tilt_rate)
            post_traj = self.BuildTrajectory(laser_scan_goal.max_angle, self.zero_tilt_position, self.target_angular_rate)
            result = self.RunTrajectory(prep_traj)
            if (result):
                rospy.loginfo("LaserScanAction prep completed")
            else:
                rospy.logerr("Unable to complete LaserScanAction")
                self.server.set_aborted()
            self.laser_scans = []
            self.active = True
            result = self.RunTrajectory(scan_traj)
            if (result):
                rospy.loginfo("LaserScanAction scan completed")
            else:
                rospy.logerr("Unable to complete LaserScanAction")
                self.server.set_aborted()
            self.active = False
            print "Scan action recorded " + str(len(self.laser_scans)) + " scans during the scan process"
            result = self.RunTrajectory(post_traj)
            if (result):
                rospy.loginfo("LaserScanAction post completed")
                self.server.set_succeeded()
            else:
                rospy.logerr("Unable to complete LaserScanAction")
                self.server.set_aborted()

    def CheckSafetyBounds(self, tilt):
        if (tilt == None):
            return False
        if (tilt > self.max_tilt_position or tilt < self.min_tilt_position):
            return False
        return True

    def BuildTrajectory(self, start_tilt, end_tilt, angular_rate):
        steps_per_sec = 50.0
        steptime = 1.0 / steps_per_sec
        trajectory_states = []
        angle = abs(end_tilt - start_tilt)
        nominal_time = angle / angular_rate
        if (abs(angular_rate) > self.target_angular_rate):
            rospy.logerr("Commanded scan velocity (rad/s) is greater than limit")
            nominal_time = angle / self.target_angular_rate
        steps = int(math.ceil(nominal_time * steps_per_sec))
        if (steps == 1):
            return [[end_tilt, steptime]]
        elif (steps < 1):
            rospy.logwarn("Trajectory built has zero states")
            return []
        else:
            for step in range(steps):
                p = float(step) / float(steps - 1)
                tilt = ((end_tilt - start_tilt) * p) + start_tilt
                trajectory_states.append([tilt, steptime])
            return trajectory_states

    def RunTrajectory(self, trajectory, debug=False):
        if (debug):
            rospy.logwarn("Running in debug mode, trajectory will not execute")
            for state in trajectory:
                print state
            return False
        for [tilt, timestep] in trajectory:
            self.tilt_command_pub.publish(tilt)
            rospy.sleep(timestep)
        if (self.last_tilt_state.error > self.error_threshold):
            self.tilt_command_pub.publish(self.last_tilt_state.current_pos)
            return False
        else:
            return True

if __name__ == '__main__':
    rospy.init_node("drchubo_laser_controller")
    rospy.loginfo("Loading LaserScanController...")
    tilt_controller_prefix = rospy.get_param("~tilt_controller_prefix", "/lidar_tilt_controller")
    zero_tilt_position = rospy.get_param("~zero_tilt_position", 0.0)
    max_tilt_position = rospy.get_param("~max_tilt_position", (math.pi / 4.0))
    min_tilt_position = rospy.get_param("~min_tilt_position", -(math.pi / 4.0))
    target_angular_rate = rospy.get_param("~target_angular_rate", (math.pi / 4.0))
    error_threshold = rospy.get_param("~error_threshold", (math.pi / 36.0))
    laser_topic = rospy.get_param("~laser_topic", "laser_scan")
    LSC = LaserScanController(tilt_controller_prefix, zero_tilt_position, max_tilt_position, min_tilt_position, laser_topic, target_angular_rate, error_threshold)
    retry_rate = rospy.Rate(1.0)
    result = False
    while not result and not rospy.is_shutdown():
        result = LSC.SelfTest()
        retry_rate.sleep()
    result = False
    rospy.loginfo("Starting LaserScanAction server...")
    result = LSC.RunActionServer()
    if (not result):
        rospy.logwarn("PointHeadController actionserver stopped, attempting to safely shutdown")
    rospy.loginfo("...Operations complete, PointHeadController safed and shutdown")
