/*
ROS code:
---------
Copyright (c) 2012, Calder Phillips-Grafflin, WPI DRC Team
(2-clause BSD)

ACH code:
---------
Copyright (c) 2012, Daniel M. Lofaro
(3-clause BSD)
*/

// Basic includes
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sstream>
#include <errno.h>
#include <fcntl.h>
#include <assert.h>
#include <unistd.h>
#include <pthread.h>
#include <ctype.h>
#include <string.h>
#include <stdbool.h>
#include <inttypes.h>

// ROS includes
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "rosgraph_msgs/Clock.h"

// Hubo kinematic state includes
#include "hubo_robot_msgs/JointCommandState.h"
#include "hubo_robot_msgs/JointControllerState.h"

// HUBO-ACH includes
#include "ach.h"
#include "hubo_components/hubo.h"

#define FT_LW 1
#define FT_RW 2
#define FT_LA 0
#define FT_RA 3
#define LEFT_IMU 0
#define RIGHT_IMU 1
#define BODY_IMU 2

// Defines
#define FT_SENSOR_COUNT 4
#define IMU_SENSOR_COUNT 3

// ACH channel
ach_channel_t chan_hubo_state;

// Clock storage
double g_last_clock;

// ROS publishers
ros::Publisher g_hubo_state_pub;
ros::Publisher g_hubo_clock_pub;

// Joint name and mapping storage
std::vector<std::string> g_joint_names;
std::map<std::string,int> g_joint_mapping;

// Debug mode switch
int hubo_debug = 0;

// Given an index, return the matching joint name
inline std::string IndexToJointName(int joint_index)
{
    for (unsigned int i = 0; i < g_joint_names.size(); i++)
    {
        if (g_joint_mapping[g_joint_names[i]] == joint_index)
        {
            return g_joint_names[i];
        }
    }
    return std::string("unknown");
}

// Convert HUBO-ACH state to an ROS JointCommandState message
bool ACHtoHuboState(struct hubo_state * robot_state)
{
    if (robot_state != NULL)
    {
        // Publish clock info
        rosgraph_msgs::Clock clock_state;
        clock_state.clock = ros::Time(robot_state->time);
        g_hubo_clock_pub.publish(clock_state);
        // Make new message
        hubo_robot_msgs::JointCommandState joint_msg;
        // Read through the two Hubo-ACH structs into the joint_message
        for (int i = 0; i < HUBO_JOINT_COUNT; i++)
        {
            if (robot_state->joint != NULL)
            {
                // Copy an individual joint
                hubo_robot_msgs::JointControllerState joint_state;
                // Populate joint status (what it's doing, not how it's doing)
                joint_state.joint_name = IndexToJointName(i);
                joint_state.set_point = robot_state->joint[i].ref;
                joint_state.process_value = robot_state->joint[i].pos;
                joint_state.process_value_dot = robot_state->joint[i].vel;
                joint_state.error = robot_state->joint[i].ref - robot_state->joint[i].pos;
                joint_state.time_step = robot_state->time - g_last_clock;
                joint_state.command = robot_state->joint[i].duty;
                joint_state.p = NAN;
                joint_state.i = NAN;
                joint_state.d = NAN;
                joint_state.i_clamp = NAN;
                joint_state.current = robot_state->joint[i].cur;
                joint_state.temperature = robot_state->joint[i].tmp;
                // Populate joint status (how it's doing, not what it's doing)
                joint_state.active = robot_state->joint[i].active;
                joint_state.zeroed = robot_state->joint[i].zeroed;
                joint_state.driver_on = robot_state->status[i].driverOn;
                joint_state.controller_on = robot_state->status[i].ctrlOn;
                joint_state.mode = robot_state->status[i].mode;
                joint_state.limit_switch = robot_state->status[i].limitSwitch;
                joint_state.home_flag = robot_state->status[i].homeFlag;
                joint_state.jammed = robot_state->status[i].jam;
                joint_state.pwm_saturated = robot_state->status[i].pwmSaturated;
                joint_state.big_error = robot_state->status[i].bigError;
                joint_state.encoder_error = robot_state->status[i].encError;
                joint_state.driver_fault = robot_state->status[i].driverFault;
                joint_state.motor_fail_0 = robot_state->status[i].motorFail0;
                joint_state.motor_fail_1 = robot_state->status[i].motorFail1;
                joint_state.under_min_position = robot_state->status[i].posMinError;
                joint_state.over_max_position = robot_state->status[i].posMaxError;
                joint_state.over_velocity = robot_state->status[i].velError;
                joint_state.over_acceleration = robot_state->status[i].accError;
                joint_state.over_temperature = robot_state->status[i].tempError;
                // Store it
                joint_msg.joint_names.push_back(joint_state.joint_name);
                joint_msg.state.push_back(joint_state);
            }
            else
            {
                ROS_ERROR("*** WARNING - NULL joint state received! ***\n");
            }
        }
        joint_msg.header.frame_id = std::string("");
        joint_msg.header.stamp = ros::Time::now();
        g_hubo_state_pub.publish(joint_msg);
        g_last_clock = robot_state->time;
        return true;
    }
    else
    {
        ROS_ERROR("*** WARNING - NULL hubo state received! ***\n");
        return false;
    }
}


int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "hubo_ros_feedback");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    ROS_INFO("Initializing ACH-to-ROS bridge [feedback]");
    g_last_clock = 0.0;
    // Load joint names and mappings
    XmlRpc::XmlRpcValue joint_names;
    if (!nhp.getParam("joints", joint_names))
    {
        ROS_FATAL("No joints given. (namespace: %s)", nhp.getNamespace().c_str());
        exit(1);
    }
    if (joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_FATAL("Malformed joint specification.  (namespace: %s)", nhp.getNamespace().c_str());
        exit(1);
    }
    for (unsigned int i = 0; i < joint_names.size(); ++i)
    {
        XmlRpc::XmlRpcValue &name_value = joint_names[i];
        if (name_value.getType() != XmlRpc::XmlRpcValue::TypeString)
        {
            ROS_FATAL("Array of joint names should contain all strings.  (namespace: %s)", nhp.getNamespace().c_str());
            exit(1);
        }
        g_joint_names.push_back((std::string)name_value);
    }
    // Gets the hubo ach index for each joint
    for (size_t i = 0; i < g_joint_names.size(); ++i)
    {
        std::string ns = std::string("mapping/") + g_joint_names[i];
        int h;
        nhp.param(ns + "/huboachid", h, -1);
        g_joint_mapping[g_joint_names[i]] = h;
    }
    // initialize HUBO-ACH feedback channel
    int r = ach_open(&chan_hubo_state, HUBO_CHAN_STATE_NAME , NULL);
    assert(ACH_OK == r);
    // Initialize HUBO-ACH message structs
    struct hubo_state H_state;
    memset(&H_state, 0, sizeof(H_state));
    size_t fs;
    ROS_INFO("HUBO-ACH channels loaded");
    // Construct ROS publisher
    g_hubo_state_pub = nh.advertise<hubo_robot_msgs::JointCommandState>(nh.getNamespace() + "/hubo_state", 1);
    g_hubo_clock_pub = nh.advertise<rosgraph_msgs::Clock>("clock", 1);
    ROS_INFO("ROS publishers loaded");
    // Loop
    while (ros::ok())
    {
        // Get latest state from HUBO-ACH
        r = ach_get(&chan_hubo_state, &H_state, sizeof(H_state), &fs, NULL, ACH_O_WAIT);
        // Process the received state
        if(ACH_OK != r)
        {
            ROS_ERROR("ACH not OK");
        }
        else if(fs == sizeof(H_state))
        {
            if(ACHtoHuboState(&H_state))
            {
                ;
            }
            else
            {
                ROS_ERROR("Invalid state recieved from Hubo");
            }
        }
        else
        {
            ROS_ERROR("Received hubo state is the wrong size");
        }
        ros::spinOnce();
    }
    //Satisfy the compiler
    return 0;
}
