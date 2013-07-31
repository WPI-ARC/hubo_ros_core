#!/usr/bin/python

#################################################
#                                               #
#   Calder Phillips-Grafflin - WPI/ARC Lab      #
#                                               #
#   Head pointing controller for DRCHubo        #
#                                               #
#   This provides two interfaces:               #
#                                               #
#   1.  PointHeadAction to point the head at a  #
#       given point in the world.               #
#                                               #
#   2.  GeneratePanoramaAction to generate a    #
#       panorama of the robot's environment.    #
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
import dynamixel_msgs.msg as dmms

class PointHeadController:

    def __init__(self, pan_controller_prefix, tilt_controller_prefix, zero_pan_position, zero_tilt_position, safe_pan_position, safe_tilt_position, target_angular_rate, error_threshold):
        self.zero_pan_position = zero_pan_position
        self.zero_tilt_position = zero_tilt_position
        self.safe_pan_position = safe_pan_position
        self.safe_tilt_position = safe_tilt_position
        self.target_angular_rate = target_angular_rate
        self.error_threshold = error_threshold
        self.tf_listener = tf.TransformListener()
        self.running = True
        self.last_pan_state = None
        self.last_tilt_state = None
        rospy.loginfo("Configuring PointHeadController...")
        rospy.loginfo("pan_controller_prefix = " + pan_controller_prefix)
        rospy.loginfo("tilt_controller_prefix = " + tilt_controller_prefix)
        self.pan_subscriber = rospy.Subscriber(pan_controller_prefix + "/state", dmms.JointState, self.pan_state_cb)
        self.tilt_subscriber = rospy.Subscriber(tilt_controller_prefix + "/state", dmms.JointState, self.tilt_state_cb)
        rospy.loginfo("Loaded subscribers to head controller states")
        retry_rate = rospy.Rate(1.0)
        while ((self.last_pan_state == None or self.last_tilt_state == None) and not rospy.is_shutdown()):
            rospy.loginfo("Waiting for states from head controllers...")
            retry_rate.sleep()
        rospy.loginfo("...Head controllers ready")
        rospy.loginfo("Loading head controller command publishers...")
        self.pan_command_pub = rospy.Publisher(pan_controller_prefix + "/command", Float64)
        self.tilt_command_pub = rospy.Publisher(tilt_controller_prefix + "/command", Float64)
        rospy.loginfo("...Loaded command publishers")
        rospy.loginfo("...PointHeadController setup finished")

    def pan_state_cb(self, msg):
        self.last_pan_state = msg

    def tilt_state_cb(self, msg):
        self.last_tilt_state = msg

    def BringUp(self):
        rospy.loginfo("Bringing up the PointHeadController to zero state")
        if (self.last_tilt_state.current_pos < -1.1):
            rospy.loginfo("Unstowing head first...")
            unstow_traj = self.BuildTrajectory(self.last_pan_state.current_pos, self.last_tilt_state.current_pos, self.last_pan_state.current_pos, -1.1, self.target_angular_rate)
            result = self.RunTrajectory(unstow_traj)
            if (result):
                rospy.loginfo("Head unstowed")
            else:
                rospy.logerr("HEAD UNABLE TO UNSTOW - will try again")
                return result
        bringup_traj = self.BuildTrajectory(self.last_pan_state.current_pos, self.last_tilt_state.current_pos, self.zero_pan_position, self.zero_tilt_position, self.target_angular_rate)
        result = self.RunTrajectory(bringup_traj)
        if (result):
            rospy.loginfo("Head brought up to the zero position")
        else:
            rospy.logerr("HEAD UNABLE TO ZERO - will try again")
        return result

    def RunActionServer(self):
        self.server = actionlib.SimpleActionServer('drchubo_point_head', hrms.PointHeadAction, self.execute_point_head, False)
        self.server.start()
        safety_rate = rospy.Rate(10.0)
        while not rospy.is_shutdown() and self.running:
            safety_rate.sleep()
        del(self.server)
        return False

    def execute_point_head(self, point_head_goal):
        [pan, tilt] = self.ComputePointingAngle(point_head_goal)
        safe = self.CheckSafetyBounds(pan, tilt)
        if not safe:
            rospy.logerr("Aborting PointHeadAction Goal due to safety violations")
            self.server.set_aborted()
        else:
            point_traj = self.BuildTrajectory(self.last_pan_state.current_pos, self.last_tilt_state.current_pos, pan, tilt, self.target_angular_rate, point_head_goal.min_duration.to_sec())
            result = self.RunTrajectory(stow_traj)
            if (result):
                rospy.loginfo("PointHeadAction completed")
                self.server.set_succeeded()
            else:
                rospy.logerr("Unable to complete PointHeadAction")
                self.server.set_aborted()

    def ComputePointingAngle(self, point_head_goal):
        if ("optical" in point_head_goal.pointing_frame and point_head_goal.pointing_axis.z == 0.0):
            rospy.logerr("PointHead specified using an optical frame but pointing axis is not Z")
            self.server.set_aborted()
            return [None, None]
        elif (point_head_goal.pointing_axis.x == 0.0):
            rospy.logerr("PointHead specified using a physical frame but pointing axis is not X")
            self.server.set_aborted()
            return [None, None]
        real_pointing_frame = self.GetRealPointingFrame(point_head_goal.pointing_frame)
        try:
            [tft, tfr] = self.tf_listener.lookupTransform(real_pointing_frame, point_head_goal.target.header.frame_id)
            target_frame_pose = PoseFromTransform(TransformFromComponents(tft,tfr))
            target_pose = Pose()
            target_pose.orientation.w = 1.0
            target_pose.position.x = point_head_goal.target.point.x
            target_pose.position.y = point_head_goal.target.point.y
            target_pose.position.z = point_head_goal.target.point.z
            pointing_frame_target_pose = ComposePoses(target_frame_pose, target_pose)
            print "Computed pose of target point:"
            print "X: " + str(pftp.position.x)
            print "Y: " + str(pftp.position.y)
            print "Z: " + str(pftp.position.z)
            target_range = math.sqrt((pftp.position.x ** 2) + (pftp.position.y ** 2) + (ptfp.position.z ** 2))
            target_pan = math.atan2(pftp.position.y, pftp.position.x)
            target_tilt = math.asin(pftp.position.z / target_range)
            print "Computed pan/tilt adjustments:"
            print "Pan change: " + str(target_pan)
            print "Tilt change: " + str(target_tilt)
            real_pan = self.last_pan_state.current_pos + target_pan
            real_tilt = self.last_tilt_state.current_pos + target_tilt
            print "Computed pan/tilt targets:"
            print "Pan: " + str(real_pan)
            print "Tilt: " + str(real_tilt)
            return [real_pan, real_tilt]
        except:
            rospy.logerr("Unable to compute pointing - this is probably because a frame doesn't exist")
            self.server.set_aborted()
            return [None, None]

    def GetRealPointingFrame(self, frame_name):
        return "/Body_NK2"

    def CheckSafetyBounds(self, pan, tilt):
        if (pan == None or tilt == None):
            return False
        min_pan = - math.pi
        max_pan = math.pi
        min_tilt = -1.1
        max_tilt = 0.5
        if (pan > max_pan or pan < min_pan):
            return False
        if (tilt > max_tilt or tilt < min_tilt):
            return False
        return True

    def BringDown(self):
        rospy.loginfo("Bringing up the PointHeadController to zero state")
        bringdown_traj = self.BuildTrajectory(self.last_pan_state.current_pos, self.last_tilt_state.current_pos, self.safe_pan_position, self.safe_tilt_position, self.target_angular_rate)
        result = self.RunTrajectory(bringdown_traj)
        if (result):
            rospy.loginfo("Head brought down to the safe position")
        else:
            rospy.logerr("HEAD UNABLE TO ZERO - will try again")
            return result
        rospy.loginfo("Tucking head into stow position")
        stow_traj = self.BuildTrajectory(self.last_pan_state.current_pos, self.last_tilt_state.current_pos, self.safe_pan_position, self.safe_tilt_position - 0.1, self.target_angular_rate)
        result = self.RunTrajectory(stow_traj)
        if (result):
            rospy.loginfo("Head stowed")
        else:
            rospy.logerr("HEAD UNABLE TO STOW - will not try again")
        return True

    def BuildTrajectory(self, start_pan, start_tilt, end_pan, end_tilt, angular_rate, execution_time=None):
        steps_per_sec = 50.0
        steptime = 1.0 / steps_per_sec
        trajectory_states = []
        start_quaternion = self.PanTiltToQuaternion(start_pan, start_tilt)
        end_quaternion = self.PanTiltToQuaternion(end_pan, end_tilt)
        angle = self.QuaternionDistance(start_quaternion, end_quaternion)
        nominal_time = angle / angular_rate
        if (execution_time != None):
            if (execution_time / angle > angular_rate):
                rospy.logwarn("Desired execution time would require exceeding velocity bounds")
            else:
                nominal_time = execution_time
        steps = int(math.ceil(nominal_time * steps_per_sec))
        if (steps == 1):
            return [[end_pan, end_tilt, steptime]]
        elif (steps < 1):
            return []
        else:
            for step in range(steps):
                p = float(step) / float(steps - 1)
                pan = ((end_pan - start_pan) * p) + start_pan
                tilt = ((end_tilt - start_tilt) * p) + start_tilt
                trajectory_states.append([pan, tilt, steptime])
            return trajectory_states

    def PanTiltToQuaternion(self, pan, tilt):
        pan = quaternion_about_axis(pan, (0,0,1))
        tilt = quaternion_about_axis(tilt, (0,1,0))
        return NormalizeQuaternion(ComposeQuaternions(pan, tilt))

    def QuaternionDistance(self, q1, q2):
        dot_product = (q1[0]* q2[0]) + (q1[1] * q2[1]) + (q1[2] * q2[2]) + (q1[3] * q2[3])
        temp_value = 2 * (dot_product ** 2) - 1
        return math.acos(temp_value)

    def RunTrajectory(self, trajectory, debug=False):
        if (debug):
            rospy.logwarn("Running in debug mode, trajectory will not execute")
            for state in trajectory:
                print state
            return False
        rospy.loginfo("Executing head trajectory...")
        for [pan, tilt, timestep] in trajectory:
            self.pan_command_pub.publish(pan)
            self.tilt_command_pub.publish(tilt)
            rospy.sleep(timestep)
        if (self.last_pan_state.error > self.error_threshold or self.last_tilt_state.error > self.error_threshold):
            self.pan_command_pub.publish(self.last_pan_state.current_pos)
            self.tilt_command_pub.publish(self.last_tilt_state.current_pos)
            return False
        else:
            return True

if __name__ == '__main__':
    rospy.init_node("drchubo_head_controller")
    rospy.loginfo("Loading PointHeadController...")
    pan_controller_prefix = rospy.get_param("~pan_controller_prefix", "/head_pan_controller")
    tilt_controller_prefix = rospy.get_param("~tilt_controller_prefix", "/head_tilt_controller")
    zero_pan_position = rospy.get_param("~zero_pan_position", 0.0)
    zero_tilt_position = rospy.get_param("~zero_tilt_position", 0.0)
    safe_pan_position = rospy.get_param("~safe_pan_position", 0.0)
    safe_tilt_position = rospy.get_param("~safe_tilt_position", -1.1)
    target_angular_rate = rospy.get_param("~target_angular_rate", (math.pi / 4.0))
    error_threshold = rospy.get_param("~error_threshold", (math.pi / 36.0))
    PHC = PointHeadController(pan_controller_prefix, tilt_controller_prefix, zero_pan_position, zero_tilt_position, safe_pan_position, safe_tilt_position, target_angular_rate, error_threshold)
    retry_rate = rospy.Rate(1.0)
    result = False
    while not result and not rospy.is_shutdown():
        result = PHC.BringUp()
        retry_rate.sleep()
    result = False
    result = PHC.RunActionServer()
    if (not result):
        rospy.logwarn("PointHeadController actionserver stopped, attempting to safely shutdown")
    result = False
    while not result and not rospy.is_shutdown():
        result = PHC.BringDown()
        retry_rate.sleep()
    rospy.loginfo("...Operations complete, PointHeadController safed and shutdown")
