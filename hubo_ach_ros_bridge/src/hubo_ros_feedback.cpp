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
#include "/usr/local/include/hubo.h"

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

// Global variables
ach_channel_t chan_hubo_state;

ros::Publisher g_hubo_state_pub;
ros::Publisher g_hubo_clock_pub;

// Debug mode switch
int hubo_debug = 0;

// Index->Joint name mapping
char *joint_names[] = {"HPY", "not in urdf NKY/HNY", "HNR", "HNP", "LSP", "LSR", "LSY", "LEP", "LWY", "not in urdf LWR", "LWP", "RSP", "RSR", "RSY", "REP", "RWY", "not in urdf RWR", "RWP", "not in ach1", "LHY", "LHR", "LHP", "LKP", "LAP", "LAR", "not in ach2", "RHY", "RHR", "RHP", "RKP", "RAP", "RAR", "RF1", "RF2", "RF3", "RF4", "RF5", "LF1", "LF2", "LF3", "LF4", "LF5", "unknown1", "unknown2", "unknown3", "unknown4", "unknown5", "unknown6", "unknown7", "unknown8"};

//Convert HUBO-ACH state to an ROS HuboState message
bool ACHtoHuboState(struct hubo_state * robot_state)
{
    if (robot_state != NULL)
    {
        //Publish clock info
        rosgraph_msgs::Clock clock_state;
        clock_state.clock = ros::Time(robot_state->time);
        g_hubo_clock_pub.publish(clock_state);
        //Make new message
        hubo_robot_msgs::JointCommandState joint_msg;
        //Read through the two Hubo-ACH structs into the joint_message
        for (int i = 0; i < HUBO_JOINT_COUNT; i++)
        {
            if (robot_state->joint != NULL)
            {
                //Copy an individual joint
                hubo_robot_msgs::JointControllerState joint_state;
                joint_state.joint_name = std::string(joint_names[i]);
                joint_state.set_point = robot_state->joint[i].ref;
                joint_state.process_value = robot_state->joint[i].pos;
                joint_state.process_value_dot = robot_state->joint[i].vel;
                joint_state.error = robot_state->joint[i].ref - robot_state->joint[i].pos;
                joint_state.time_step = NAN;
                joint_state.command = NAN;
                joint_state.p = NAN;
                joint_state.i = NAN;
                joint_state.d = NAN;
                joint_state.i_clamp = NAN;
                joint_state.current = robot_state->joint[i].cur;
                joint_state.temperature = robot_state->joint[i].tmp;
                joint_state.active = (int)robot_state->joint[i].active;
                joint_state.zeroed = (int)robot_state->joint[i].zeroed;
                joint_msg.joint_names.push_back(std::string(joint_names[i]));
                joint_msg.state.push_back(joint_state);
            }
            else
            {
                ROS_ERROR("*** WARNING - NULL joint state received! ***\n");
            }
        }
        joint_msg.header.frame_id = std::string("/hubo_base_link");
        joint_msg.header.stamp = ros::Time::now();
        g_hubo_state_pub.publish(joint_msg);
        return true;
    }
    else
    {
        ROS_ERROR("*** WARNING - NULL hubo state received! ***\n");
        return false;
    }
}


//NEW MAIN LOOP
int main(int argc, char **argv)
{
    ROS_INFO("Initializing ACH-to-ROS bridge");
    //initialize HUBO-ACH feedback channel
    int r = ach_open(&chan_hubo_state, HUBO_CHAN_STATE_NAME , NULL);
    assert(ACH_OK == r);
    //initialize HUBO-ACH message structs
    struct hubo_state H_state;
    memset(&H_state, 0, sizeof(H_state));
    size_t fs;
    ROS_INFO("HUBO-ACH channels loaded");
    //initialize ROS node
    ros::init(argc, argv, "hubo_ros_feedback");
    ros::NodeHandle nh;
    //construct ROS publisher
    g_hubo_state_pub = nh.advertise<hubo_robot_msgs::JointCommandState>("hubo/HuboState", 1);
    g_hubo_clock_pub = nh.advertise<rosgraph_msgs::Clock>("clock", 1);
    ROS_INFO("ROS publishers loaded");
    //Loop
    while (ros::ok())
    {
        //Get latest state from HUBO-ACH
        r = ach_get(&chan_hubo_state, &H_state, sizeof(H_state), &fs, NULL, ACH_O_WAIT);

        if(ACH_OK != r)
        {
            if(hubo_debug)
            {
                ROS_DEBUG("State ini r = %i",r);
            }
        }
        else
        {
            //ROS_INFO("fs : %d, sizeof(H_state) : %d\n",fs,sizeof(H_state));
            assert(sizeof(H_state) == fs);
        }

        if(ACHtoHuboState(&H_state))
        {
            ;
        }
        else
        {
            ROS_ERROR("*** Invalid state recieved from HUBO! ***");
        }
        ros::spinOnce();
    }
    //Satisfy the compiler
    return 0;
}
