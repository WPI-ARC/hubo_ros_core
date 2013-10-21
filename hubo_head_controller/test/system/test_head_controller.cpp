/**
 * \file test_head_controller.cpp
 * \brief
 *
 * \author Andrew Price
 * \date July 18, 2013
 *
 * \copyright
 *
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Humanoid Robotics Lab Georgia Institute of Technology
 * Director: Mike Stilman http://www.golems.org
 *
 * This file is provided under the following "BSD-style" License:
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the following
 * conditions are met:
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <hubo_robot_msgs/HokuyoScanAction.h>
#include <hubo_robot_msgs/PointHeadAction.h>

bool gSpoofDaemon = true;

hubo_robot_msgs::HokuyoScanParameters createSlowScan()
{
	hubo_robot_msgs::HokuyoScanParameters params;
	params.degreesPerSecond = 5;
	params.minTheta = -M_PI/4.0;
	params.maxTheta = 0.0;//M_PI/4.0;
	return params;
}

hubo_robot_msgs::HokuyoScanParameters createFastScan()
{
	hubo_robot_msgs::HokuyoScanParameters params;
	params.degreesPerSecond = 25;
	params.minTheta = -M_PI/4.0;
	params.maxTheta = M_PI/4.0;
	return params;
}

hubo_robot_msgs::HokuyoScanParameters createLittleScan()
{
	hubo_robot_msgs::HokuyoScanParameters params;
	params.degreesPerSecond = 10;
	params.minTheta = -M_PI/8.0;
	params.maxTheta = M_PI/8.0;
	return params;
}

hubo_robot_msgs::HokuyoScanParameters createBigScan()
{
	hubo_robot_msgs::HokuyoScanParameters params;
	params.degreesPerSecond = 10;
	params.minTheta = -M_PI/3.0;
	params.maxTheta = M_PI/3.0;
	return params;
}

bool testScanClient(hubo_robot_msgs::HokuyoScanParameters params)
{
	hubo_robot_msgs::HokuyoScanGoal goal;
	goal.Parameters = params;
	// create the action client
	// true causes the client to spin its own thread
	actionlib::SimpleActionClient<hubo_robot_msgs::HokuyoScanAction> ac("hokuyo_scan_action", true);

	ROS_INFO("Waiting for action server to start.");
	// wait for the action server to start
	ac.waitForServer(); //will wait for infinite time

	ROS_INFO("Action server started, sending goal.");

	// send a goal to the action
	ac.sendGoal(goal);

	// TODO: Check Ach data here

	//wait for the action to return
	bool finished_before_timeout = ac.waitForResult(ros::Duration(25.0));

	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = ac.getState();
		ROS_INFO("Action finished: %s",state.toString().c_str());
	}
	else
	{
		ROS_INFO("Action did not finish before the time out.");
		return false;
	}
	return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_head_server");
	ROS_INFO("Started test_head_server.");
	ros::NodeHandle nh;

	nh.param<bool>("spoof_daemon", gSpoofDaemon, true);

	testScanClient(createSlowScan());
	testScanClient(createFastScan());
	testScanClient(createLittleScan());
	testScanClient(createBigScan());

	ros::shutdown();
	return 0;
}
