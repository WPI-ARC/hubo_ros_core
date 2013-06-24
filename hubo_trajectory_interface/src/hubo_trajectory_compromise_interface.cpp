/*
 * Copyright (c) 2013, Calder Phillips-Grafflin (WPI) and M.X. Grey (Georgia Tech), Drexel DARPA Robotics Challenge team
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
#include <stdlib.h>
#include <vector>
// System includes to handle safe shutdown
#include <signal.h>
// ROS & includes
#include <ros/ros.h>
// Boost includes
#include <boost/thread.hpp>
// Message and action includes for Hubo actions
#include <trajectory_msgs/JointTrajectory.h>
#include <hubo_robot_msgs/JointTrajectoryState.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <rosgraph_msgs/Clock.h>
// Includes for ACH and hubo-motion-rt
#include <ach.h>
#include "hubo_components/hubo.h"
#include "hubo_components/motion-trajectory.h"

using std::cout;
using std::endl;

// ACH channels
ach_channel_t chan_traj_cmd;
ach_channel_t chan_traj_state;
ach_channel_t chan_hubo_state;
//ach_channel_t chan_hubo_ref_filter;
ach_channel_t chan_hubo_ctrl_state_main;
ach_channel_t chan_hubo_ctrl_state_pub;

// In debug mode we do not take into 
// account flushing (premature abortion of the trajectory execution)
bool debug_interface = true;

/*
 * These are the two values we don't quite know how to set - we actually want these
 * to be "slow" - i.e. small trajectory chunks at a comparatively slow rate, since
 * it allows for a useful "cancelling" of the trajectory to be done by simply not
 * sending any more chunks without needing explicit support for this in hubo motion.
 *
 * These values may need to be experimentally determined, and the SPIN_RATE parameter
 * may need to dynamically change based on the timings on the incoming trajectory.
*/
#define MAX_TRAJ_LENGTH 10 //Number of points in each trajectory chunk
double SPIN_RATE = 40.0; //Rate in hertz at which to send trajectory chunks

// Index->Joint name mapping (the index in this array matches the numerical index of the joint name in Hubo-Ach as defined in hubo.h
const char* joint_names[] = {"HPY", "not in urdf1", "HNR", "HNP", "LSP", "LSR", "LSY", "LEP", "LWY", "not in urdf2", "LWP", "RSP", "RSR", "RSY", "REP", "RWY", "not in urdf3", "RWP", "not in ach1", "LHY", "LHR", "LHP", "LKP", "LAP", "LAR_dummy", "not in ach2", "RHY", "RHR", "RHP", "RKP", "RAP", "RAR_dummy", "not in urdf4", "not in urdf5", "not in urdf6", "not in urdf7", "not in urdf8", "not in urdf9", "not in urdf10", "not in urdf11", "not in urdf12", "not in urdf13", "unknown1", "unknown2", "unknown3", "unknown4", "unknown5", "unknown6", "unknown7", "unknown8"};

// Trajectory storage
std::vector<std::string> g_joint_names;
std::vector< std::vector<trajectory_msgs::JointTrajectoryPoint> > g_trajectory_chunks;
int g_tid = 0;

// Publisher and subscriber
ros::Publisher g_state_pub;
ros::Publisher g_clock_pub;
ros::Subscriber g_traj_sub;

// Thread for listening to the hubo state and republishing it
boost::thread* pub_thread;
boost::thread* traj_thread;

// Execution state
bool g_running = false;
bool g_next_chunk_sent = false;
bool g_wait_for_new_state = false;

/*
 * Signal handler to safely shutdown the node
*/
void shutdown(int signum)
{
    ROS_INFO("Attempting to shutdown node...");
    if (signum == SIGINT)
    {
        g_trajectory_chunks.clear();
        ROS_INFO("Starting safe shutdown...");
        
        //pub_thread->join();
        ROS_INFO("All threads done");

        ach_close(&chan_traj_cmd);
        ach_close(&chan_traj_state);
        ach_close(&chan_hubo_state);
        ach_close(&chan_hubo_ctrl_state_main);
        ach_close(&chan_hubo_ctrl_state_pub);
        ROS_INFO("ach channels closed shutting down!");

        ros::shutdown();
    }
}

/*
 * Given a string name of a joint, it looks it up in the list of joint names
 * to determine the joint index used in hubo ach for that joint. If the name
 * can't be found, it returns -1.
*/
int IndexLookup(const std::string& joint_name)
{
    // Find the Hubo joint name [used in hubo.h, with some additions for joints that don't map directly]
    // and the relevant index to map from the ROS trajectories message to the hubo-motion struct
    bool match = false;
    int best_match = -1;
    //See if we've got a matching joint name, and if so, return the
    //relevant index so we can map it into the hubo struct
    for (int i = 0; i < HUBO_JOINT_COUNT; i++)
    {
        if (strcmp(joint_name.c_str(), joint_names[i]) == 0)
        {
            match = true;
            best_match = i;
            break;
        }
    }
    if (match)
    {
        return best_match;
    }
    else
    {
        return -1;
    }
}

/*
 * Sets a null trajectory in the channel
*/
void resetTrajectoryChannel()
{
    hubo_traj_t ach_traj;
    ach_traj.trajID = -1;
    ach_traj.endTime = -1;
    ach_put( &chan_traj_cmd, &ach_traj, sizeof(ach_traj) );

    hubo_traj_output_t H_output;
    memset(&H_output, 0, sizeof(hubo_traj_output_t));

    size_t fs; int i=0;

    H_output.status = TRAJ_RUNNING;

    while( H_output.status != TRAJ_COMPLETE )
    {
        ach_get( &chan_traj_state, &H_output, sizeof(H_output), &fs, NULL, ACH_O_LAST );

        if( i++ == 1000 )
            break;
    }
}

/*
 * Takes the current "trajectory chunk" and sends it to hubo-motion over
 * hubo ach channels. Because the trajectory chunk has been reprocessed,
 * the joint indices in the trajectory chunk match the indicies used by
 * inside hubo ach, and no further remapping is needed.
*/
void sendTrajectory( const std::vector<trajectory_msgs::JointTrajectoryPoint>& processed_traj )
{
    if ( processed_traj.empty() )
    {
        return;
    }
    else
    {
        // Make a new hubo_traj_t, and fill it in with the command data we've processed
        hubo_traj_t ach_traj;
        memset(&ach_traj, 0, sizeof(ach_traj));

        for (size_t p = 0; p < processed_traj.size(); p++)
        {
            for (int i = 0; i < HUBO_JOINT_COUNT; i++)
            {
                ach_traj.joint[i].position[p] = processed_traj[p].positions[i];
                ach_traj.joint[i].velocity[p] = processed_traj[p].velocities[i];
                ach_traj.joint[i].acceleration[p] = processed_traj[p].accelerations[i];
                // cout << ach_traj.joint[i].position[p] << " " ;
                // cout << ach_traj.joint[i].acceleration[p] << " " ;
            }
            //cout << endl;
            ach_traj.time[p] = processed_traj[p].time_from_start.toSec();
            //cout << "time from start : " << processed_traj[p].time_from_start.toSec() << endl;
        }
        ach_traj.endTime = processed_traj.back().time_from_start.toSec();
        ach_traj.trajID = g_tid;
        cout << "Send trajectory chunk, size : " << processed_traj.size() << " , end time : " << ach_traj.endTime << endl;
        ach_put( &chan_traj_cmd, &ach_traj, sizeof(ach_traj) );
        g_tid++;
    }
}

/**************************************************************************************************************************
 * POTENTIALLY NEEDS MAJOR CHANGES TO HUBO-MOTION-RT
 *
 * This function re-orders a given JointTrajectoryPoint in the commanded trajectory to do two things:
 *
 * 1) Provide a command to every joint, including those that may not match the URDF
 *    This is done by ordering the data with the same indexing as that used directly
 *    by the Hubo. This effectively handles the conversion between trajectories in
 *    ROS that use joint names to identify active joints, and trajectories used with
 *    Hubo-Ach that use indexing to match values with joints.
 *
 * 2) Set all "unused" joints, i.e. those not being actively commanded, to their last
 *    setpoint as reported by the trajectory controller & hubo-ach. Once again, this
 *    provides support for trajectories with limited active joints.
 *
 * In order for this to be possible, ideally we would have 6 pieces of information per joint:
 *
 * 1) Current setpoint position
 * 2) Current setpoint velocity
 * 3) Current setpoint acceleration
 * 4) Current actual position
 * 5) Current actual velocity
 * 6) Current actual acceleration
 *
 * Of those, 4 and 5 are provided from the hubo-state channel, and 1 is provided from the
 * hubo-ref and/or hubo-ref-filter channels.
 *
 * There are two courses of action here - either, (a) fields 2,3, and 6 can be added to hubo-ach's
 * reporting of hubo state in hubo motion, or (b) the interface can be changed to use hubo-ach and
 * hubo-motion together with several assumptions to not require the missing information.
 *
 * From our perspective, (a) is a better option, as it maintains functional equivalence with other
 * robots such as the PR2 and doesn't require any major assumptions.
 *
 * However, (b) may be easier to implement, BUT, it requires two major assumptions:
 *
 * 1) All uncommanded joints (i.e. those not in the current trajectory)
 *    have zero desired velocity and zero desirec acceleration.
 *    [This lets us avoid the need for data on vel. and accel. setpoints]
 *
 * 2) All trajectories must end at zero velocity
 *    [This means we can specifically assume that any velocity at the end of a
 *     trajectory is error velocity]
 *
 * In this version of the file, we demonstrate what we would do in case (b) where no changes to hubo-motion
 * necessary and we change the interface slightly as a result. A consequnce of this is than not all data
 * reported from this node is accurate, as it doesn't all exist in the first place!
 *************************************************************************************************************************/
trajectory_msgs::JointTrajectoryPoint processPoint( const trajectory_msgs::JointTrajectoryPoint& raw, hubo_ctrl_state_t* cur_commands )
{
    //cout << "process point" << endl;
    trajectory_msgs::JointTrajectoryPoint processed;
    processed.positions.resize(HUBO_JOINT_COUNT);
    processed.velocities.resize(HUBO_JOINT_COUNT);
    processed.accelerations.resize(HUBO_JOINT_COUNT);

    // Set time to be the same
    processed.time_from_start = raw.time_from_start;

    if (g_joint_names.size() != raw.positions.size())
    {
        ROS_ERROR("Stored joint names and received joint commands do not match");

        for (int i = 0; i < HUBO_JOINT_COUNT; i++)
        {
            processed.positions[i] = cur_commands->requested_pos[i];
            processed.velocities[i] = cur_commands->requested_vel[i];
            processed.accelerations[i] = cur_commands->requested_acc[i];
        }
        return processed;
    }
    else
    {
        // Remap the provided joint trajectory point to the Hubo's joint indices
        // First, fill in everything with data from the current Hubo state
        for (int i = 0; i < HUBO_JOINT_COUNT; i++)
        {
            processed.positions[i] = cur_commands->requested_pos[i];
            processed.velocities[i] = cur_commands->requested_vel[i];
            processed.accelerations[i] = cur_commands->requested_acc[i];
        }
        // Now, overwrite with the commands in the current trajectory
        for (size_t i = 0; i<raw.positions.size(); i++)
        {
            int index = IndexLookup(g_joint_names[i]);
            if (index != -1)
            {
                processed.positions[index] = raw.positions[i];
                processed.velocities[index] = raw.velocities[i];                            
                processed.accelerations[index] = raw.accelerations[i];
            }
        }

        return processed;
    }
}


/*
 * Reprocesses the current "trajectory chunk" one point at a time to make it safe
 * to send to hubo motion that assumes all joints will be commanded/populated.
 *
 * Once a chunk has been reprocessed, it is removed from the stored list of chunks.
*/
std::vector<trajectory_msgs::JointTrajectoryPoint> processTrajectory( hubo_ctrl_state_t* cur_commands )
{
    std::vector<trajectory_msgs::JointTrajectoryPoint> processed;

    if ( g_trajectory_chunks.empty() )
    {
        return processed;
    }
    else
    {
        // Grab the next chunk to execute
        std::vector<trajectory_msgs::JointTrajectoryPoint> cur_set = g_trajectory_chunks[0];

        for (size_t i=0; i<cur_set.size(); i++)
        {
            const trajectory_msgs::JointTrajectoryPoint& processed_point = processPoint( cur_set[i], cur_commands );
            processed.push_back( processed_point );
        }

        // Remove the current chunk from storage now that we've used it
        g_trajectory_chunks.erase( g_trajectory_chunks.begin(), g_trajectory_chunks.begin()+1 );

        return processed;
    }
}

/*
 * Callback to chunk apart the trajectory into chunks that will fit over hubo-ach
 *
 * For a trajectory of N points and a chunk size of M points, this will produce
 * ceil(N/M) chunks.
 *
 * An important part of this chunking is that each chunk is retimed from the end time
 * of the previous chunk. Otherwise, as ROS trajectories are timed via absolute time
 * from start, the trajectory chunks would "drift into the future".
 */
void trajectoryCB( const trajectory_msgs::JointTrajectory& traj )
{
    //cout << "trajectoryCB : " << traj << endl;
    // Callback to chunk and save incoming trajectories
    // Before we do anything, check if the trajectory is empty - this is a special "stop" value that flushes the current stored trajectory
    if (traj.points.size() == 0 && !debug_interface)
    {
        g_trajectory_chunks.clear();
        ROS_INFO("Flushing current trajectory");
        return;
    }
    else if (traj.points.size() == 0)
    {
        ROS_WARN("Execution cancelled, NOT ABORTING DUE TO DEBUG MODE");
        return;
    }
    ROS_INFO("Reprocessing trajectory with %ld elements into chunks", traj.points.size());

    // First, chunk the trajectory into parts that can be sent over ACH channels to hubo-motion-rt
    std::vector< std::vector<trajectory_msgs::JointTrajectoryPoint> > new_chunks;
    unsigned int i = 0;
    ros::Duration base_time(0.0);

    while ( i < traj.points.size() )
    {
        std::vector<trajectory_msgs::JointTrajectoryPoint> new_chunk;
        unsigned int index = 0;
        while ( i < traj.points.size() && index < MAX_TRAJ_LENGTH )
        {
            // Make sure the JointTrajectoryPoint gets retimed to match its new trajectory chunk
            trajectory_msgs::JointTrajectoryPoint cur_point = traj.points[i];

            // Retime based on the end time of the previous trajectory chunk
            //cout << "cur_point.time_from_start : " << cur_point.time_from_start << endl;
            cur_point.time_from_start = cur_point.time_from_start - base_time;
            //cout << "cur_point.time_from_start : " << cur_point.time_from_start << endl;

            // Make sure position, velocity, and acceleration are all the same length
            int size = cur_point.positions.size();
            cur_point.velocities.resize(size);
            cur_point.accelerations.resize(size);

            // Store it
            new_chunk.push_back( cur_point );
            index++;
            i++;
        }

        ROS_INFO("Assembled a new trajectory chunk with %ld elements", new_chunk.size());

        if( !new_chunk.empty() )
        {
            // Save the ending time to use for the next chunk
            base_time = traj.points[i-1].time_from_start;
            //base_time = new_chunk.back().time_from_start;
            // Store it
            new_chunks.push_back(new_chunk);
        }
    }

    ROS_INFO("Trajectory reprocessed into %ld chunks", new_chunks.size());

    // Second, store those chunks - first, we flush the stored trajectory
    g_trajectory_chunks.clear();
    g_joint_names.clear();
    g_trajectory_chunks = new_chunks;
    g_joint_names = traj.joint_names;
//    cout << "leave trajectory callback" << endl;
}

/*
 * This runs in a second thread in the background.
 *
 * This thread grabs the latest states from the hubo's joints and joint setpoints
 * and repacks them into the ROS messages sent to the trajectory action server.
 *
 * NOTE: This thread uses ACH_0_WAIT as a delay operation to synchronize against
 * hubo ach, and thus it doesn't need a explicity sleep() or wait operation.
 *
 * NOTE: This implements option (b) from above!
 */
void publishLoop()
{
    ROS_INFO("publishLoop");
    ach_status_t r;
    r = ach_open( &chan_hubo_ctrl_state_pub,  CTRL_CHAN_STATE, NULL );
    if (r != ACH_OK)
    {
        ROS_FATAL("Could not open ACH channel: CTRL_CHAN_STATE !");
        exit(1);
    }

    hubo_ctrl_state_t H_ctrl_state;
    memset(&H_ctrl_state, 0, sizeof(H_ctrl_state));

    //ROS_INFO("should enter pub loop");

    // Loop until node shutdown
    while (ros::ok())
    {
        size_t fs;

        // Get latest state from HUBO-MOTION (this is used to populate the desired values)
        r = ach_get( &chan_hubo_ctrl_state_pub,  &H_ctrl_state, sizeof(H_ctrl_state), &fs, NULL, ACH_O_LAST );

        if( r != ACH_OK && r != ACH_MISSED_FRAME && r != ACH_STALE_FRAMES )
        {
            ROS_ERROR("get ach channel for H_ctrl_state failed in [publishing loop] : %s" , ach_result_to_string(r) );
            //continue;
        }
        else if (fs != sizeof(H_ctrl_state))
        {
            //ROS_ERROR("Hubo ref size error! [publishing loop] with ach channel %s" , ach_result_to_string(r));
            continue;
        }

        //cout << "Read state correctly in the publish loop" << endl;

        // Publish the latest hubo state back out
        hubo_robot_msgs::JointTrajectoryState cur_state;
        cur_state.header.stamp = ros::Time::now();
        // Set the names
        cur_state.joint_names = g_joint_names;
        unsigned int num_joints = cur_state.joint_names.size();
        // Make the empty states
        trajectory_msgs::JointTrajectoryPoint cur_setpoint;
        trajectory_msgs::JointTrajectoryPoint cur_actual;
        trajectory_msgs::JointTrajectoryPoint cur_error;
        // Resize the states
        cur_setpoint.positions.resize(num_joints);
        cur_setpoint.velocities.resize(num_joints);
        cur_setpoint.accelerations.resize(num_joints);
        cur_actual.positions.resize(num_joints);
        cur_actual.velocities.resize(num_joints);
        cur_actual.accelerations.resize(num_joints);
        cur_error.positions.resize(num_joints);
        cur_error.velocities.resize(num_joints);
        cur_error.accelerations.resize(num_joints);
        // Fill in the setpoint and actual & calc the error in the process
        for (unsigned int i = 0; i < num_joints; i++)
        {
            // Fill in the setpoint and actual data
            // Values that we don't have data for are set to NAN
            int hubo_index = IndexLookup(cur_state.joint_names[i]);

            cur_setpoint.positions[i]       = H_ctrl_state.requested_pos[hubo_index];
            cur_setpoint.velocities[i]      = H_ctrl_state.requested_vel[hubo_index];
            cur_setpoint.accelerations[i]   = H_ctrl_state.requested_acc[hubo_index];

            cur_actual.positions[i]         = H_ctrl_state.actual_pos[hubo_index];
            cur_actual.velocities[i]        = H_ctrl_state.actual_vel[hubo_index];
            cur_actual.accelerations[i]     = H_ctrl_state.actual_acc[hubo_index];
            // Calc the error
            cur_error.positions[i] = cur_setpoint.positions[i] - cur_actual.positions[i];
            cur_error.velocities[i] = cur_setpoint.velocities[i] - cur_actual.velocities[i];
            cur_error.accelerations[i] =  cur_setpoint.accelerations[i] - cur_actual.accelerations[i];
        }
        // Pack them together
        cur_state.desired = cur_setpoint;
        cur_state.actual = cur_actual;
        cur_state.error = cur_error;

        // Publish State
        g_state_pub.publish( cur_state );

        hubo_state_t H_state;
        r = ach_get( &chan_hubo_state,  &H_state, sizeof(H_state), &fs, NULL, ACH_O_LAST );

        // Publish Time
        rosgraph_msgs::Clock clockmsg;
        clockmsg.clock = ros::Time( H_state.time );
        g_clock_pub.publish( clockmsg );
        //ROS_INFO("TIME is : %f sec", H_state.time );
    }
}

void trajectoryStatusLoop()

{
    ach_status_t r;
    size_t fs;
    hubo_traj_output_t H_output;
    memset(&H_output, 0, sizeof(hubo_traj_output_t));

    while( 1 )
    {
        // Reset the trajectory channel when execution is finished
        r = ach_get( &chan_traj_state, &H_output, sizeof(H_output), &fs, NULL, ACH_O_WAIT );

        if( r != ACH_OK && r != ACH_MISSED_FRAME && r != ACH_STALE_FRAMES )
        {
            ROS_ERROR("get ach channel for H_output failed in [sending loop] : %s", ach_result_to_string(r) );
        }
        else if( fs != sizeof(H_output) )
        {
            ROS_ERROR("Hubo output size error! [sending loop] with %s", ach_result_to_string(r) );
        }
        else if( g_running && H_output.status == TRAJ_COMPLETE && g_trajectory_chunks.empty() )
        {
            ROS_INFO( "trajectory completed, reset trajectory channel" );
            //resetTrajectoryChannel();
            g_running = false;
        }
        else if( H_output.status == TRAJ_COMPLETE && g_trajectory_chunks.size() > 0 )
        {
            ROS_INFO( "trajectory chunk complete!" );
            g_next_chunk_sent = false;
        }
        else if( H_output.status == TRAJ_RUNNING )
        {
            //cout << "trajectory running" << endl;
            g_running = true;
            g_wait_for_new_state = true;

            if( r != ACH_OK )
            {
                ROS_INFO("error receiving traj status : %s" , ach_result_to_string(r) );
            }
        }
    }
}

int main(int argc, char** argv)
{
    std::cout << "Starting JointTrajectoryAction controller interface node..." << std::endl;
    ros::init(argc, argv, "hubo_joint_trajectory_controller_interface_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    ROS_INFO("Attempting to start JointTrajectoryAction controller interface...");

    // Get all the active joint names
    XmlRpc::XmlRpcValue joint_names;

    if (!nhp.getParam("/hubo_fullbody_controller/hubo_fullbody_controller_node/joints", joint_names))
    {
        ROS_FATAL("No joints given. (namespace: %s)", nhp.getNamespace().c_str());
        exit(1);
    }
    if (joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_FATAL("Malformed joint specification.  (namespace: %s)", nhp.getNamespace().c_str());
        exit(1);
    }
    for (int i = 0; i < joint_names.size(); ++i)
    {
        XmlRpc::XmlRpcValue &name_value = joint_names[i];
        if (name_value.getType() != XmlRpc::XmlRpcValue::TypeString)
        {
            ROS_FATAL("Array of joint names should contain all strings.  (namespace: %s)", nhp.getNamespace().c_str());
            exit(1);
        }
        g_joint_names.push_back((std::string)name_value);
    }
    // Register a signal handler to safely shutdown the node
    signal(SIGINT, shutdown);
    ach_status_t r;
    // Set up the ACH channels to and from hubo-motion-rt
//    char command[100];
//    sprintf(command, "ach -1 -C %s -m 10 -n 1000000 -o 666", HUBO_TRAJ_CHAN);
//    system(command);
//    sprintf(command, "ach -1 -C %s -o 666", HUBO_TRAJ_STATE_CHAN);
//    system(command);

    //initialize HUBO-ACH reference channel
    r = ach_open( &chan_hubo_state, HUBO_CHAN_STATE_NAME , NULL );
    //r = ach_open(&chan_hubo_ref_filter, HUBO_CHAN_REF_NAME , NULL);
    if (r != ACH_OK)
    {
        ROS_FATAL("Could not open ACH channel: HUBO_CHAN_STATE_NAME !");
        exit(1);
    }

    r = ach_open( &chan_hubo_ctrl_state_main, CTRL_CHAN_STATE , NULL );
    //r = ach_open(&chan_hubo_ref_filter, HUBO_CHAN_REF_NAME , NULL);
    if (r != ACH_OK)
    {
        ROS_FATAL("Could not open ACH channel: CTRL_CHAN_STATE !");
        exit(1);
    }
    // Make sure the ACH channels to hubo motion are opened properly
    r = ach_open( &chan_traj_cmd, HUBO_TRAJ_CHAN, NULL);
    if (r != ACH_OK)
    {
        ROS_FATAL("Could not open ACH channel: HUBO_TRAJ_CHAN !");
        exit(1);
    }
    r = ach_open( &chan_traj_state, HUBO_TRAJ_STATE_CHAN, NULL);
    if (r != ACH_OK)
    {
        ROS_FATAL("Could not open ACH channel: HUBO_TRAJ_STATE_CHAN !");
        exit(1);
    }

    ROS_INFO("Opened ACH channels to hubo-motion-rt");
    // Set up state publisher
    std::string pub_path = nh.getNamespace() + "/state";
    g_state_pub = nh.advertise<hubo_robot_msgs::JointTrajectoryState>(pub_path, 1);

    // Set up clock publisher
    g_clock_pub = nh.advertise<rosgraph_msgs::Clock>("/clock", 1);

    // Spin up the thread for getting data from hubo and publishing it
    pub_thread = new boost::thread(&publishLoop);

    // Spin up the thread for getting the trajectory execution status
    traj_thread = new boost::thread(&trajectoryStatusLoop);

    // Set up the trajectory subscriber
    std::string sub_path = nh.getNamespace() + "/command";
    g_traj_sub = nh.subscribe( sub_path, 1, trajectoryCB );
    ROS_INFO("Loaded trajectory interface to hubo-motion-rt");
    // Spin until killed
    
    size_t fs;

    hubo_ctrl_state_t H_ctrl_state;
    memset(&H_ctrl_state, 0, sizeof(H_ctrl_state));

    //resetTrajectoryChannel();
    
    g_running = false;
    g_next_chunk_sent = false;
    g_wait_for_new_state = false;

    while (ros::ok())
    {
        // Get latest reference from HUBO-ACH (this is used to populate the uncommanded joints!)
        r = ach_get( &chan_hubo_ctrl_state_main, &H_ctrl_state, sizeof(H_ctrl_state), &fs, NULL, ACH_O_LAST );

        if( r != ACH_OK && r != ACH_MISSED_FRAME && r != ACH_STALE_FRAMES )
        {
            ROS_ERROR("get ach channel for H_ctrl_state failed in [sending loop] : %s", ach_result_to_string(r) );
        }
        else if (fs != sizeof(H_ctrl_state))
        {
            ROS_ERROR("Hubo state size error! [sending loop] with %s", ach_result_to_string(r) );
        }
        else
        {
            // Send the latest trajectory chunk (this does nothing if we have nothing to send)
            // only send the next chunk
            if( !g_running || !g_next_chunk_sent )
            {
                // cout << "processing trajectory" << endl;
                // Reprocess the current trajectory chunk (this does nothing if we have nothing to send)
                const std::vector<trajectory_msgs::JointTrajectoryPoint>& cleaned_trajectory = processTrajectory( &H_ctrl_state );

                if ( !cleaned_trajectory.empty() )
                {
                    //SPIN_RATE = 1.0 / (cleaned_trajectory.back().time_from_start.toSec());

                    sendTrajectory( cleaned_trajectory );

                    if( g_running )
                        g_next_chunk_sent = true;
                   
                    g_wait_for_new_state = true;
                }
             }
        }

//        for (int i=0;i<HUBO_JOINT_COUNT;i++)
//            ROS_INFO("Trajectory error : %d, %f", i, H_output.error[i] );

//        ROS_INFO("Trajectory id : %d", H_output.trajID );
//        ROS_INFO("Trajectory state channel in [sending loop] : %s", ach_result_to_string(r) );  
//        ROS_INFO("Trajectory size of : %d", sizeof(H_output) );

        // Wait long enough before sending the next one and receiving the running signal
        ros::spinOnce();
        ros::Rate looprate(SPIN_RATE);
        looprate.sleep();
    }
    // Make the compiler happy
    return 0;
}
