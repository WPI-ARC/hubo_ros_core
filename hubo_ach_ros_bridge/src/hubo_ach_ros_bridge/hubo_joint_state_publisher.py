#!/usr/bin/python

import rospy
import xml.dom.minidom
import subprocess
from hubo_robot_msgs.msg import *
from sensor_msgs.msg import *
from math import pi

class JointStatePublisher:

    def __init__(self, description_file):
        robot = xml.dom.minidom.parseString(description_file).getElementsByTagName('robot')[0]
        self.free_joints = {}
        self.latest_state = None
        self.last_time = rospy.get_time()
        # Create all the joints based off of the URDF and assign them joint limits
        # based on their properties
        for child in robot.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue
            if child.localName == 'joint':
                jtype = child.getAttribute('type')
                if jtype == 'fixed':
                    continue
                name = child.getAttribute('name')
                if jtype == 'continuous':
                    minval = -pi
                    maxval = pi
                else:
                    limit = child.getElementsByTagName('limit')[0]
                    minval = float(limit.getAttribute('lower'))
                    maxval = float(limit.getAttribute('upper'))

                if minval > 0 or maxval < 0:
                    zeroval = (maxval + minval)/2
                else:
                    zeroval = 0

                joint = {'min':minval, 'max':maxval, 'zero':zeroval, 'value':zeroval }
                self.free_joints[name] = joint
        #Setup the HuboState subscriber
        self.hubo_sub = rospy.Subscriber("hubo/HuboState", JointCommandState, self.hubo_cb)
        #Setup the joint state publisher
        self.hubo_pub = rospy.Publisher("/joint_states", JointState)

    def hubo_cb(self, msg):
        new_state = {}
        try:
            assert(len(msg.joint_names) == len(msg.state))
            for index in range(len(msg.joint_names)):
                new_state[msg.joint_names[index]] = msg.state[index]
            self.latest_state = new_state
        except:
            rospy.logerr("*** Malformed HuboState! ***")
        self.last_time = rospy.get_time()

    def loop(self, hz=10.0):
        r = rospy.Rate(hz) 
        
        # Publish Joint States
        while not rospy.is_shutdown():
            #Check to warn if the data from the Hubo is too old
            time_difference = rospy.get_time() - self.last_time
            threshold = 0.05
            if (time_difference > threshold):
                rospy.logwarn("Hubo messages are old")
            #Convert the latest state of the robot if it is available
            if (self.latest_state != None):
                msg = JointState()
                msg.header.stamp = rospy.Time.now()
                #Convert the HuboState to a series of joint states
                for joint_name in self.latest_state.keys():
                    #Determine if the joint is FREE or DEPENDENT
                    if (joint_name in self.free_joints.keys()):
                        msg.name.append(joint_name)
                        msg.position.append(self.latest_state[joint_name].process_value)
                        msg.velocity.append(self.latest_state[joint_name].process_value_dot)
                        msg.effort.append(self.latest_state[joint_name].current)
                    else:
                        rospy.logwarn("Joint in update message not found in the URDF")
                #Check for joints in the URDF that are not in the HuboState
                for joint_name in self.free_joints:
                    if (joint_name not in self.latest_state.keys()):
                        rospy.logwarn("Joint in the URDF not in the update message")
                        msg.name.append(joint_name)
                        msg.position.append(self.free_joints[joint_name]['zero'])
                self.hubo_pub.publish(msg)
            else:
                rospy.logerr("No valid message received from the Hubo yet")
                msg = JointState()
                msg.header.stamp = rospy.Time.now()
                for joint_name in self.free_joints:
                    msg.name.append(joint_name)
                    msg.position.append(self.free_joints[joint_name]['zero'])
                self.hubo_pub.publish(msg)
            #Spin
            r.sleep()

if __name__ == '__main__':
    description_file = rospy.get_param("robot_description")
    rospy.init_node('hubo_joint_state_publisher')
    #rospy.loginfo("Loading description file: " + description_file)
    jsp = JointStatePublisher(description_file)
    jsp.loop()
