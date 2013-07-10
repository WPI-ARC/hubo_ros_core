/*
 * Actionlib JointTrajectoryAction interface to the Hubo robot.
 *
 * This executable provides an externally-facing ActionServer interface for JointTrajectoryAction goals
 * which it connects internally to the hubo_trajectory_*_interface that passes trajectories to the
 * hubo-motion-rt system using ACH channels.
 *
 * Licensing:
 * -------------
 *
 * Copyright (c) 2013, Calder Phillips-Grafflin, WPI/Drexel DARPA Robotics Challenge team
 * 2-Clause BSD
 *
 * Derived from the JointTrajectoryAction interface to the PR2 written by Stuart Glaser
 *
 * Original license:
 *
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
// System includes to handle safe shutdown
#include <signal.h>
// ROS & Actionlib includes
#include <ros/ros.h>
#include <actionlib/server/action_server.h>
// Message and action includes for Hubo actions
#include <trajectory_msgs/JointTrajectory.h>
#include <hubo_robot_msgs/JointTrajectoryAction.h>
#include <hubo_robot_msgs/JointTrajectoryState.h>

using std::cout;
using std::endl;

const double DEFAULT_GOAL_THRESHOLD = 0.1;

class HuboJointTrajectoryServer
{
private:

    typedef actionlib::ActionServer<hubo_robot_msgs::JointTrajectoryAction> HJTAS; // "Hubo Joint Trajectory Action Server"
    typedef HJTAS::GoalHandle HTGH; // "Hubo Trajectory Goal Handle"

    ros::NodeHandle node_;
    HJTAS action_server_;
    ros::Publisher pub_interface_command_;
    ros::Subscriber sub_interface_state_;
    ros::Timer watchdog_timer_;
    bool has_active_goal_;
    HTGH active_goal_;
    trajectory_msgs::JointTrajectory current_traj_;
    std::vector<std::string> joint_names_;
    std::map<std::string,double> goal_constraints_;
    std::map<std::string,double> trajectory_constraints_;
    double goal_time_constraint_;
    double stopped_velocity_tolerance_;
    bool traj_ends_stopped;
    hubo_robot_msgs::JointTrajectoryStateConstPtr last_interface_state_;

    /*
     * Determine if two vectors are equal. Used to check joint names stored in the node against joint names in a new trajectory
    */
    inline static bool setsEqual(const std::vector<std::string> &a, const std::vector<std::string> &b)
    {
        // First, check to make sure that the sets are the same size
        if (a.size() != b.size())
        {
            ROS_ERROR("Set a size: %lu, set b size: %lu", a.size(), b.size());
            return false;
        }
        // Now, make sure a is a subset of b (i.e. make sure every element of a is in b)
        for (size_t i = 0; i < a.size(); ++i)
        {
            if (count(b.begin(), b.end(), a[i]) != 1)
            {
                ROS_ERROR("Element a[%lu] (value %s) is not in b", i, a[i].c_str());
                return false;
            }
        }
        // Now, make sure b is a subset of a (i.e. make sure every element of b is in a)
        for (size_t i = 0; i < b.size(); ++i)
        {
            if (count(a.begin(), a.end(), b[i]) != 1)
            {
                ROS_ERROR("Element b[%lu] (value %s) is not in a", i, b[i].c_str());
                return false;
            }
        }
        // If a is a subset of b and b is a subset of a, then a = b
        return true;
    }

    /*
     * Watchdog timer callback to abort a goal if we aren't hearing back from the trajectory controller
    */
    void watchdog(const ros::TimerEvent &e)
    {
        ros::Time now = ros::Time::now();
        // Aborts the active goal if the controller does not appear to be active.
        if (has_active_goal_)
        {
            bool should_abort = false;
            if (!last_interface_state_)
            {
                should_abort = true;
                ROS_WARN("Aborting goal because we have never heard a controller state message.");
            }
            else if ((now - last_interface_state_->header.stamp) > ros::Duration(5.0))
            {
                should_abort = true;
                ROS_WARN("Aborting goal because we haven't heard from the controller in %.3lf seconds", (now - last_interface_state_->header.stamp).toSec());
            }
            if (should_abort)
            {
                // Stops the controller.
                trajectory_msgs::JointTrajectory empty;
                empty.joint_names = joint_names_;
                pub_interface_command_.publish(empty);
                // Marks the current goal as aborted.
                active_goal_.setAborted();
                has_active_goal_ = false;
            }
        }
    }

    /*
     * Get a new goal, check it for safety, and send it to the trajectory controller
    */
    void goalCB(HTGH gh)
    {
        // Make sure the goal is in the future (if it starts in the past, it could be older than a newer goal that got here first)
        ros::Time now = ros::Time::now();

        if (gh.getGoal()->trajectory.points.size() > 0 && 
            now > (gh.getGoal()->trajectory.header.stamp + gh.getGoal()->trajectory.points[0].time_from_start))
        {
            ROS_ERROR("Goal timing in the past");
            gh.setRejected();
            return;
        }
        // Ensures that the joints in the goal match the joints we are commanding (fail if they don't)
        if (!setsEqual(joint_names_, gh.getGoal()->trajectory.joint_names))
        {
            ROS_ERROR("Joints on incoming goal don't match our joints");
            gh.setRejected();
            return;
        }
        // Cancels the currently active goal (if it exists) to preempt it with a new trajectory
        if (has_active_goal_)
        {
            // Stops the controller.
            trajectory_msgs::JointTrajectory empty;
            empty.joint_names = joint_names_;
            pub_interface_command_.publish(empty);
            // Marks the current goal as canceled.
            active_goal_.setCanceled();
            has_active_goal_ = false;
        }
        // Accept and send the new goal
        gh.setAccepted();
        active_goal_ = gh;
        has_active_goal_ = true;
        // Sends the trajectory along to the controller
        current_traj_ = active_goal_.getGoal()->trajectory;
        ROS_INFO("Sending goal trajectory to the execution backend");
        pub_interface_command_.publish(current_traj_);
    }

    /*
     * Cancels the current goal by sending an empty trajectory to the trajectory controller
    */
    void cancelCB(HTGH gh)
    {
        if (!has_active_goal_)
        {
            ROS_ERROR("No active goal to cancel");
        }
        if (active_goal_ == gh)
        {
            // Stops the controller.
            trajectory_msgs::JointTrajectory empty;
            empty.joint_names = joint_names_;
            pub_interface_command_.publish(empty);
            // Marks the current goal as canceled.
            active_goal_.setCanceled();
            has_active_goal_ = false;
        }
        else
        {
            ROS_ERROR("Attempted to cancel inactive goal");
        }
    }

    /*
     * Gets the current state from the trajectory controller and determines if it
     * is within acceptable bounds (i.e. if the position error is below the defined
     * threshold or if the trajectory end time has been reached) and decides if the
     * current goal has been reached or if it needs to be aborted (if the error is
     * too high or if it has gone over time).
    */
    void controllerStateCB(const hubo_robot_msgs::JointTrajectoryStateConstPtr &msg)
    {
        last_interface_state_ = msg;
        ros::Time now = ros::Time::now();
        // Nothing to do if there isn't an active goal
        if (!has_active_goal_)
        {
            return;
        }
        // Nothing to do if the current goal is empty
        if (current_traj_.points.empty())
        {
            return;
        }
        // Nothing to do if the current time is still before the first point in the trajectory
        if (now < current_traj_.header.stamp + current_traj_.points[0].time_from_start)
        {
            return;
        }
        // Throw an error message if the state message doesn't match the structure we expect
        if (!setsEqual(joint_names_, msg->joint_names))
        {
            ROS_ERROR("Joint names from the controller don't match our joint names.");
            return;
        }
        // Get the time of the last point in the current trajectory
        int last = current_traj_.points.size() - 1;
        ros::Time end_time = current_traj_.header.stamp + current_traj_.points[last].time_from_start;
        // Verifies that the controller has stayed within the trajectory constraints.
        // First, make sure that we are under the time limit
        if (now < end_time)
        {
            // Check that the controller is inside the trajectory constraints.
            for (size_t i = 0; i < msg->joint_names.size(); ++i)
            {
                double abs_error = fabs(msg->error.positions[i]);
                double constraint = trajectory_constraints_[msg->joint_names[i]];
                if (constraint >= 0 && abs_error > constraint)
                {
                    // Stops the controller.
                    trajectory_msgs::JointTrajectory empty;
                    empty.joint_names = joint_names_;
                    pub_interface_command_.publish(empty);
                    // Marks the current goal as aborted
                    active_goal_.setAborted();
                    has_active_goal_ = false;
                    ROS_WARN("Aborting because we would up outside the trajectory constraints, now < end_time!");
                    return;
                }
            }
        }
        else
        {
            // Check that we have ended inside the goal constraints
            bool inside_goal_constraints = true;
            for (size_t i = 0; i < msg->joint_names.size() && inside_goal_constraints; ++i)
            {
                double abs_error = fabs(msg->error.positions[i]);
                double goal_constraint = goal_constraints_[msg->joint_names[i]];
                ROS_DEBUG("Checking to see if joint %s ended up in constraints: current error = %f, goal_constraint = %f", msg->joint_names[i].c_str(), fabs(msg->error.positions[i]), goal_constraint);
                if (goal_constraint >= 0 && abs_error > goal_constraint)
                {
                    ROS_WARN("Joint %s ended up outside position constraint: current error = %f, goal_constraint = %f", msg->joint_names[i].c_str(), fabs(msg->error.positions[i]), goal_constraint);
                    inside_goal_constraints = false;
                }
                // It's important to be stopped if that's desired.
                // This can be set globally (i.e. all trajectories must stop)
                // or left up to each trajectory given desired velocity info
                if (traj_ends_stopped || fabs(msg->desired.velocities[i]) < 1e-6)
                {
                    if (fabs(msg->actual.velocities[i]) > stopped_velocity_tolerance_)
                    {
                        ROS_WARN("Joint %s ended up outside velocity constraint: current velocity = %f", msg->joint_names[i].c_str(), fabs(msg->actual.velocities[i]));
                        inside_goal_constraints = false;
                    }
                }
            }
            if (inside_goal_constraints)
            {
                active_goal_.setSucceeded();
                has_active_goal_ = false;
            }
            else if (now < end_time + ros::Duration(goal_time_constraint_))
            {
              // Still have some time left to make it.
            }
            else
            {
                // Stops the controller.
                trajectory_msgs::JointTrajectory empty;
                empty.joint_names = joint_names_;
                pub_interface_command_.publish(empty);
                // Marks the current goal as aborted
                active_goal_.setAborted();
                has_active_goal_ = false;
                // Get the time in secs for better error output
                double now_secs = now.toSec();
                double end_secs = end_time.toSec() + goal_time_constraint_;
                ROS_WARN("Aborting because we ran out of time, now > end_time! now = %f, end_time = %f", now_secs, end_secs);
            }
        }
    }

public:

    HuboJointTrajectoryServer(ros::NodeHandle &n, bool must_end_stopped) : node_(n),
        action_server_(node_, "joint_trajectory_action",
                       boost::bind(&HuboJointTrajectoryServer::goalCB, this, _1),
                       boost::bind(&HuboJointTrajectoryServer::cancelCB, this, _1),
                       false),
        has_active_goal_(false)
    {
        ros::NodeHandle pn("~");
        traj_ends_stopped = must_end_stopped;
        ///////////////////////////////////////////////////
        // Get joint names, constraints, and information
        // Gets all of the joint names
                XmlRpc::XmlRpcValue joint_names;
        if (!pn.getParam("joints", joint_names))
        {
            ROS_FATAL("No joints given. (namespace: %s)", pn.getNamespace().c_str());
            exit(1);
        }
        if (joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_FATAL("Malformed joint specification.  (namespace: %s)", pn.getNamespace().c_str());
            exit(1);
        }
        for (unsigned int i = 0; i < joint_names.size(); ++i)
        {
            XmlRpc::XmlRpcValue &name_value = joint_names[i];
            if (name_value.getType() != XmlRpc::XmlRpcValue::TypeString)
            {
                ROS_FATAL("Array of joint names should contain all strings.  (namespace: %s)", pn.getNamespace().c_str());
                exit(1);
            }
            joint_names_.push_back((std::string)name_value);
            //cout << joint_names_.back() << endl;
        }
        std::ostringstream strm;
        strm << "Loaded joint names from the parameter server:";
        for (unsigned int i = 0; i < joint_names_.size(); i++)
        {
            strm << " " << joint_names_[i];
        }
        ROS_INFO("%s", strm.str().c_str());
        // Get the goal time constraint (i.e. how long after the desired end time we let the goal run before aborting)
        pn.param("constraints/goal_time", goal_time_constraint_, 0.0);
        // Gets the constraints for each joint.
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            std::string ns = std::string("constraints/") + joint_names_[i];
            double g, t;
            pn.param(ns + "/goal", g, -1.0);
            pn.param(ns + "/trajectory", t, -1.0);
            goal_constraints_[joint_names_[i]] = g;
            trajectory_constraints_[joint_names_[i]] = t;
        }
        // Get the tolerance constraint for "zero" velocity
        pn.param("constraints/stopped_velocity_tolerance", stopped_velocity_tolerance_, 0.01);
        ///////////////////////////////////////////////////
        // Set up the publisher & subscriber link to the trajectory controller interface
        pub_interface_command_ = node_.advertise<trajectory_msgs::JointTrajectory>(node_.getNamespace() + "/command", 1);
        sub_interface_state_ = node_.subscribe(node_.getNamespace() + "/state", 1, &HuboJointTrajectoryServer::controllerStateCB, this);
        // Set up a watchdog timer to handle drops in the communication between this server and the trajectory controller
        watchdog_timer_ = node_.createTimer(ros::Duration(1.0), &HuboJointTrajectoryServer::watchdog, this);

        ROS_INFO("Waiting for the execution backend to start...");
        ///////////////////////////////////////////////////
        // Wait for the interface to be ready before starting the joint trajectory action server
        ros::Time started_waiting_for_controller = ros::Time::now();
        while (ros::ok() && !last_interface_state_)
        {
            ros::spinOnce();

            if (started_waiting_for_controller != ros::Time(0) && ros::Time::now() > started_waiting_for_controller + ros::Duration(30.0))
            {
                ROS_WARN("Waited for the controller for 30 seconds, but it never showed up.");
                started_waiting_for_controller = ros::Time::now();
            }
            ros::Duration(0.1).sleep();
        }
        // Once everything is ready, start the actionserver
        action_server_.start();
        ROS_INFO("JointTrajectoryAction server started");
    }

    ~HuboJointTrajectoryServer()
    {
        pub_interface_command_.shutdown();
        sub_interface_state_.shutdown();
        watchdog_timer_.stop();
    }

    /*
     * Attempts to safely shutdown the trajectory interface by aborting the current goal and telling the controller to stop
    */
    void shutdown(int signum)
    {
        if (signum == SIGINT)
        {
            // Stops the controller.
            ROS_INFO("Starting safe shutdown...");
            trajectory_msgs::JointTrajectory empty;
            empty.joint_names = joint_names_;
            pub_interface_command_.publish(empty);
            if (has_active_goal_)
            {
                // Marks the current goal as canceled.
                active_goal_.setCanceled();
                has_active_goal_ = false;
            }
            ROS_INFO("Shutting down!");
        }
    }

};

// This needs to be global so the signal handler can use it
HuboJointTrajectoryServer* g_htjs;

/*
 * Signal handler to catch SIGINT (the shutdown command) and attempt to safely shutdown the trajectory interface
*/
void shutdown(int signum)
{
    ROS_WARN("Attempting to shutdown node...");
    if (g_htjs != NULL)
    {
        g_htjs->shutdown(signum);
    }
    else
    {
        ROS_WARN("HJTAS not yet loaded, aborting the load process and shutting down");
    }
    ros::shutdown();
}

int main(int argc, char** argv)
{
    ROS_INFO("Starting JointTrajectoryAction action server node...");
    ros::init(argc, argv, "hubo_joint_trajectory_action_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle node;
    // Register a signal handler to safely shutdown the node
    signal(SIGINT, shutdown);
    ROS_INFO("Attempting to start JointTrajectoryAction action server...");
    g_htjs = new HuboJointTrajectoryServer(node, true);
    ros::spin();
    // Make the compiler happy
    return 0;
}
