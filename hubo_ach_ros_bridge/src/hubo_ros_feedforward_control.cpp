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
#include <assert.h>
#include <string>
#include <vector>

// ROS includes
#include "ros/ros.h"
#include "std_msgs/String.h"

// Hubo kinematic command includes
#include "hubo_robot_msgs/JointCommand.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"

// HUBO-ACH includes
#include "ach.h"
#include "hubo_components/ach_wrapper/ach_ros_wrapper.h"
#include "hubo_components/hubo_components/hubo.h"

// ACH bridges
ACH_ROS_WRAPPER<hubo_ref>* g_ach_bridge;

// Joint name and mapping storage
std::vector<std::string> g_joint_names;
std::map<std::string,int> g_joint_mapping;

// Signal handler to safely shutdown the node
void shutdown(int signum)
{
    ROS_INFO("Attempting to shutdown node...");
    if (signum == SIGINT)
    {
        ROS_INFO("Starting safe shutdown...");
        g_ach_bridge->CancelOperations();
        g_ach_bridge->CloseChannel();
        ROS_INFO("ACH channels closed...shutting down!");
        ros::shutdown();
    }
}

// From the name of the joint, find the corresponding joint index for the Hubo-ACH struct
int IndexLookup(std::string joint_name)
{
    for (unsigned int i = 0; i < g_joint_names.size(); i++)
    {
        if(g_joint_names[i].compare(joint_name) == 0)
        {
            return g_joint_mapping[g_joint_names[i]];
        }
    }
    return -1;
}

// Callback to convert the ROS joint commands into Hubo-ACH commands
void hubo_cb(const hubo_robot_msgs::JointCommand& msg)
{
    hubo_ref current_command;
    try
    {
        current_command = g_ach_bridge->ReadLastState();
    }
    catch (...)
    {
        ROS_ERROR("Unable to get the current state from hubo-ach");
        return;
    }
    //Add the joint values one at a time into the hubo struct
    //for each joint command, we lookup the best matching
    //joint in the header to set the index
    if (msg.command.positions.size() != msg.joint_names.size())
    {
        ROS_ERROR("Hubo JointCommand malformed!");
        return;
    }
    for (unsigned int i = 0; i < msg.command.positions.size(); i++)
    {
        int index = IndexLookup(msg.joint_names[i]);
        if (index != -1)
        {
            current_command.ref[index] = msg.command.positions[i];
        }
    }
    try
    {
        g_ach_bridge->WriteState(current_command);
    }
    catch (...)
    {
        ROS_ERROR("Unable to send new commands to hubo-ach");
    }
}

int main(int argc, char **argv)
{
    //initialize ROS node
    ros::init(argc, argv, "hubo_ros_feedforward", ros::init_options::NoSigintHandler);
    signal(SIGINT, shutdown);
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    ROS_INFO("Initializing ROS-to-ACH bridge [feedforward]");
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
    // Get which Hubo-ACH channel to write to
    std::string command_channel;
    if (!nhp.getParam("commandchannel", command_channel))
    {
        ROS_WARN("No command channel provided - using default command channel: HUBO_CHAN_REF_FILTER");
        command_channel = std::string(HUBO_CHAN_REF_FILTER_NAME);
    }
    else
    {
        if (command_channel != std::string(HUBO_CHAN_REF_NAME) && command_channel != std::string(HUBO_CHAN_REF_FILTER_NAME))
        {
            ROS_WARN("Invalid command channel provided - using default command channel: HUBO_CHAN_REF_FILTER");
            command_channel = std::string(HUBO_CHAN_REF_FILTER_NAME);
        }
        else
        {
            ROS_INFO("Using command channel: %s", command_channel.c_str());
        }
    }
    // Make the connection to Hubo-ACH
    bool ready = false;
    while (!ready)
    {
        try
        {
            g_ach_bridge = new ACH_ROS_WRAPPER<hubo_ref>(command_channel);
            ready = true;
            ROS_INFO("Loaded HUBO-ACH interface");
        }
        catch (...)
        {
            ROS_WARN("Could not open ACH interface...retrying in 5 seconds...");
            ros::Duration(5.0).sleep();
        }
    }
    // Construct ROS Subscriber
    ros::Subscriber hubo_command_sub = nh.subscribe(nh.getNamespace() + "/hubo_command", 1, hubo_cb);
    ROS_INFO("Waiting for joint commands to Hubo...");
    // Spin until shutdown
    ros::spin();
    // Satisfy the compiler
    return 0;
}
