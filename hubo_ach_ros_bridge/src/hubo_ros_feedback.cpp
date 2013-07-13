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
#include "rosgraph_msgs/Clock.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/WrenchStamped.h"

// Hubo-specific message includes
#include "hubo_robot_msgs/JointCommandState.h"
#include "hubo_robot_msgs/JointControllerState.h"

// HUBO-ACH includes
#include "ach.h"
#include "hubo_ach_ros_bridge/ach_ros_wrapper.h"
#include "hubo_components/hubo.h"

// Sensor index definitions
hubo_imu_index_t imu_indices;
hubo_sensor_index_t sensor_indices;

// Sensor frame definitions
std::string imu_frame = "/body_imu_sensor_frame";
std::string left_tilt_frame = "/left_tilt_sensor_frame";
std::string right_tilt_frame = "/right_tilt_sensor_frame";
std::string left_wrist_ft_frame = "/left_wrist_ft_sensor_frame";
std::string right_wrist_ft_frame = "/right_wrist_ft_sensor_frame";
std::string left_ankle_ft_frame = "/left_ankle_ft_sensor_frame";
std::string right_ankle_ft_frame = "/right_ankle_ft_sensor_frame";

// ACH channel
ach_channel_t chan_hubo_state;

// Clock storage
double g_last_clock;

// ROS publishers
// Joints
ros::Publisher g_hubo_state_pub;
// Clock
ros::Publisher g_hubo_clock_pub;
// IMUs
ros::Publisher g_body_imu_pub;
ros::Publisher g_left_foot_imu_pub;
ros::Publisher g_right_foot_imu_pub;
// F-T Sensors
ros::Publisher g_left_ankle_ft_pub;
ros::Publisher g_right_ankle_ft_pub;
ros::Publisher g_left_wrist_ft_pub;
ros::Publisher g_right_wrist_ft_pub;

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
        ach_close(&chan_hubo_state);
        ROS_INFO("ACH channels closed...shutting down!");
        ros::shutdown();
    }
}

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
bool ACHtoHuboState(hubo_state robot_state)
{
    // Publish clock info
    rosgraph_msgs::Clock clock_state;
    clock_state.clock = ros::Time(robot_state.time);
    g_hubo_clock_pub.publish(clock_state);
    // Publish joint info
    hubo_robot_msgs::JointCommandState joint_msg;
    // Read through the two Hubo-ACH structs into the joint_message
    for (int i = 0; i < HUBO_JOINT_COUNT; i++)
    {
        // Copy an individual joint
        hubo_robot_msgs::JointControllerState joint_state;
        // Populate joint status (what it's doing, not how it's doing)
        joint_state.joint_name = IndexToJointName(i);
        joint_state.set_point = robot_state.joint[i].ref;
        joint_state.process_value = robot_state.joint[i].pos;
        joint_state.process_value_dot = robot_state.joint[i].vel;
        joint_state.error = robot_state.joint[i].ref - robot_state.joint[i].pos;
        joint_state.time_step = robot_state.time - g_last_clock;
        joint_state.command = robot_state.joint[i].duty;
        joint_state.p = NAN;
        joint_state.i = NAN;
        joint_state.d = NAN;
        joint_state.i_clamp = NAN;
        joint_state.current = robot_state.joint[i].cur;
        joint_state.temperature = robot_state.joint[i].tmp;
        // Populate joint status (how it's doing, not what it's doing)
        joint_state.active = robot_state.joint[i].active;
        joint_state.zeroed = robot_state.joint[i].zeroed;
        joint_state.driver_on = robot_state.status[i].driverOn;
        joint_state.controller_on = robot_state.status[i].ctrlOn;
        joint_state.mode = robot_state.status[i].mode;
        joint_state.limit_switch = robot_state.status[i].limitSwitch;
        joint_state.home_flag = robot_state.status[i].homeFlag;
        joint_state.jammed = robot_state.status[i].jam;
        joint_state.pwm_saturated = robot_state.status[i].pwmSaturated;
        joint_state.big_error = robot_state.status[i].bigError;
        joint_state.encoder_error = robot_state.status[i].encError;
        joint_state.driver_fault = robot_state.status[i].driverFault;
        joint_state.motor_fail_0 = robot_state.status[i].motorFail0;
        joint_state.motor_fail_1 = robot_state.status[i].motorFail1;
        joint_state.under_min_position = robot_state.status[i].posMinError;
        joint_state.over_max_position = robot_state.status[i].posMaxError;
        joint_state.over_velocity = robot_state.status[i].velError;
        joint_state.over_acceleration = robot_state.status[i].accError;
        joint_state.over_temperature = robot_state.status[i].tempError;
        // Store it
        joint_msg.joint_names.push_back(joint_state.joint_name);
        joint_msg.state.push_back(joint_state);
    }
    joint_msg.header.frame_id = std::string("");
    joint_msg.header.stamp = ros::Time(robot_state.time);
    g_hubo_state_pub.publish(joint_msg);
    g_last_clock = robot_state.time;
    /* Publish IMU data */
    // Publish body IMU data
    sensor_msgs::Imu body_imu;
    body_imu.header.stamp = ros::Time(robot_state.time);
    body_imu.header.frame_id = imu_frame;
    // Since we have no orientation information
    body_imu.orientation_covariance[0] = -1.0;
    // Since we have to covariance information, we leave the covariances at 0.0 (default)
    body_imu.linear_acceleration.x = robot_state.imu[IMU].a_x;
    body_imu.linear_acceleration.z = robot_state.imu[IMU].a_y;
    body_imu.linear_acceleration.y = robot_state.imu[IMU].a_z;
    body_imu.angular_velocity.x = robot_state.imu[IMU].w_x;
    body_imu.angular_velocity.y = robot_state.imu[IMU].w_y;
    body_imu.angular_velocity.z = robot_state.imu[IMU].w_z;
    g_body_imu_pub.publish(body_imu);
    // Publish left ankle tilt data
    sensor_msgs::Imu left_foot_tilt;
    left_foot_tilt.header.stamp = ros::Time(robot_state.time);
    left_foot_tilt.header.frame_id = left_tilt_frame;
    // Since we have no orientation information
    left_foot_tilt.orientation_covariance[0] = -1.0;
    // Since we have to covariance information, we leave the covariances at 0.0 (default)
    left_foot_tilt.linear_acceleration.x = robot_state.imu[TILT_L].a_x;
    left_foot_tilt.linear_acceleration.z = robot_state.imu[TILT_L].a_y;
    left_foot_tilt.linear_acceleration.y = robot_state.imu[TILT_L].a_z;
    left_foot_tilt.angular_velocity.x = robot_state.imu[TILT_L].w_x;
    left_foot_tilt.angular_velocity.y = robot_state.imu[TILT_L].w_y;
    left_foot_tilt.angular_velocity.z = robot_state.imu[TILT_L].w_z;
    g_left_foot_imu_pub.publish(left_foot_tilt);
    // Publish right ankle tilt data
    sensor_msgs::Imu right_foot_tilt;
    right_foot_tilt.header.stamp = ros::Time(robot_state.time);
    right_foot_tilt.header.frame_id = right_tilt_frame;
    // Since we have no orientation information
    right_foot_tilt.orientation_covariance[0] = -1.0;
    // Since we have to covariance information, we leave the covariances at 0.0 (default)
    right_foot_tilt.linear_acceleration.x = robot_state.imu[TILT_R].a_x;
    right_foot_tilt.linear_acceleration.z = robot_state.imu[TILT_R].a_y;
    right_foot_tilt.linear_acceleration.y = robot_state.imu[TILT_R].a_z;
    right_foot_tilt.angular_velocity.x = robot_state.imu[TILT_R].w_x;
    right_foot_tilt.angular_velocity.y = robot_state.imu[TILT_R].w_y;
    right_foot_tilt.angular_velocity.z = robot_state.imu[TILT_R].w_z;
    g_right_foot_imu_pub.publish(right_foot_tilt);
    /* Publish F-T sensor data */
    // Publish left wrist f-t data
    geometry_msgs::WrenchStamped left_wrist_ft;
    left_wrist_ft.header.stamp = ros::Time(robot_state.time);
    left_wrist_ft.header.frame_id = left_wrist_ft_frame;
    left_wrist_ft.wrench.force.x = NAN;
    left_wrist_ft.wrench.force.y = NAN;
    left_wrist_ft.wrench.force.z = robot_state.ft[HUBO_FT_L_HAND].f_z;
    left_wrist_ft.wrench.torque.x = robot_state.ft[HUBO_FT_L_HAND].m_x;
    left_wrist_ft.wrench.torque.y = robot_state.ft[HUBO_FT_L_HAND].m_y;
    left_wrist_ft.wrench.torque.z = NAN;
    g_left_wrist_ft_pub.publish(left_wrist_ft);
    // Publish right wrist f-t data
    geometry_msgs::WrenchStamped right_wrist_ft;
    right_wrist_ft.header.stamp = ros::Time(robot_state.time);
    right_wrist_ft.header.frame_id = right_wrist_ft_frame;
    right_wrist_ft.wrench.force.x = NAN;
    right_wrist_ft.wrench.force.y = NAN;
    right_wrist_ft.wrench.force.z = robot_state.ft[HUBO_FT_R_HAND].f_z;
    right_wrist_ft.wrench.torque.x = robot_state.ft[HUBO_FT_R_HAND].m_x;
    right_wrist_ft.wrench.torque.y = robot_state.ft[HUBO_FT_R_HAND].m_y;
    right_wrist_ft.wrench.torque.z = NAN;
    g_right_wrist_ft_pub.publish(right_wrist_ft);
    // Publish left ankle f-t data
    geometry_msgs::WrenchStamped left_ankle_ft;
    left_ankle_ft.header.stamp = ros::Time(robot_state.time);
    left_ankle_ft.header.frame_id = left_ankle_ft_frame;
    left_ankle_ft.wrench.force.x = NAN;
    left_ankle_ft.wrench.force.y = NAN;
    left_ankle_ft.wrench.force.z = robot_state.ft[HUBO_FT_L_FOOT].f_z;
    left_ankle_ft.wrench.torque.x = robot_state.ft[HUBO_FT_L_FOOT].m_x;
    left_ankle_ft.wrench.torque.y = robot_state.ft[HUBO_FT_L_FOOT].m_y;
    left_ankle_ft.wrench.torque.z = NAN;
    g_left_ankle_ft_pub.publish(left_ankle_ft);
    // Publish right ankle f-t data
    geometry_msgs::WrenchStamped right_ankle_ft;
    right_ankle_ft.header.stamp = ros::Time(robot_state.time);
    right_ankle_ft.header.frame_id = right_ankle_ft_frame;
    right_ankle_ft.wrench.force.x = NAN;
    right_ankle_ft.wrench.force.y = NAN;
    right_ankle_ft.wrench.force.z = robot_state.ft[HUBO_FT_R_FOOT].f_z;
    right_ankle_ft.wrench.torque.x = robot_state.ft[HUBO_FT_R_FOOT].m_x;
    right_ankle_ft.wrench.torque.y = robot_state.ft[HUBO_FT_R_FOOT].m_y;
    right_ankle_ft.wrench.torque.z = NAN;
    g_right_ankle_ft_pub.publish(right_ankle_ft);
    return true;
}

int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "hubo_ros_feedback", ros::init_options::NoSigintHandler);
    signal(SIGINT, shutdown);
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
    // Set up the ROS publishers
    ROS_INFO("Loading publishers...");
    g_hubo_state_pub = nh.advertise<hubo_robot_msgs::JointCommandState>(nh.getNamespace() + "/hubo_state", 1);
    g_hubo_clock_pub = nh.advertise<rosgraph_msgs::Clock>("clock", 1);
    g_body_imu_pub = nh.advertise<sensor_msgs::Imu>(nh.getNamespace() + "/body_imu", 1);
    g_left_foot_imu_pub = nh.advertise<sensor_msgs::Imu>(nh.getNamespace() + "/left_foot_tilt", 1);
    g_right_foot_imu_pub = nh.advertise<sensor_msgs::Imu>(nh.getNamespace() + "/right_foot_tilt", 1);
    g_left_wrist_ft_pub = nh.advertise<geometry_msgs::WrenchStamped>(nh.getNamespace() + "/left_wrist_ft", 1);
    g_right_wrist_ft_pub = nh.advertise<geometry_msgs::WrenchStamped>(nh.getNamespace() + "/right_wrist_ft", 1);
    g_left_ankle_ft_pub = nh.advertise<geometry_msgs::WrenchStamped>(nh.getNamespace() + "/left_ankle_ft", 1);
    g_right_ankle_ft_pub = nh.advertise<geometry_msgs::WrenchStamped>(nh.getNamespace() + "/right_ankle_ft", 1);
    ROS_INFO("ROS publishers loaded");
    // Make the connection to Hubo-ACH
    ACH_ROS_WRAPPER<hubo_state> ach_bridge(HUBO_CHAN_STATE_NAME);
    // Loop
    while (ros::ok())
    {
        try
        {
            hubo_state robot_state = ach_bridge.ReadNextState();
            ACHtoHuboState(robot_state);
        }
        catch (...)
        {
            ROS_ERROR("Unable to get/process state from hubo-ach");
        }
        ros::spinOnce();
    }
    //Satisfy the compiler
    return 0;
}
