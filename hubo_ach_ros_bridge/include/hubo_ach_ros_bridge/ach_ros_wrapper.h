#ifndef ACH_ROS_WRAPPER_H
#define ACH_ROS_WRAPPER_H

#include <stdlib.h>
#include <string>
#include <stdio.h>
#include <stdexcept>
#include <time.h>
#include "ros/ros.h"
#include "ach.h"

template <typename T>
class ACH_ROS_WRAPPER
{
protected:

    ach_channel_t ach_channel_;
    std::string channel_name_;
    T ach_data_;

public:

    ACH_ROS_WRAPPER(std::string ach_channel_name, bool create_channel=false, size_t channel_frames=10);

    ~ACH_ROS_WRAPPER();

    T ReadNextState();

    T ReadLastState();

    T ReadNextState(timespec wait_time);

    T ReadNextState(double wait_seconds);

    const bool WriteState(const T& data);

    const bool CancelOperations();

    const bool CloseChannel(bool destroy_channel=false);
};

template <typename T>
ACH_ROS_WRAPPER<T>::ACH_ROS_WRAPPER(std::string ach_channel_name, bool create_channel, size_t channel_frames)
{
    if (ach_channel_name == "")
    {
        ROS_FATAL("Invalid ACH channel name specified");
        throw std::invalid_argument("Invalid ACH channel name specified");
    }
    channel_name_ = ach_channel_name;
    ach_status_t status = ACH_OK;
    memset(&ach_data_, 0, sizeof(ach_data_));
    status = ach_open(&ach_channel_, channel_name_.c_str(), NULL);
    // If the channel doesn't already exist, make it
    if (status == ACH_ENOENT)
    {
        if (create_channel)
        {
            ROS_WARN("ACH channel: %s does not already exist, attempting to make it", channel_name_.c_str());
            status = ach_create(channel_name_.c_str(), channel_frames, sizeof(ach_data_), NULL);
            ROS_INFO("Set up ACH channel: %s", channel_name_.c_str());
            status = ach_open(&ach_channel_, channel_name_.c_str(), NULL);
        }
        else
        {
            throw std::invalid_argument("ACH channel: " + channel_name_ + " does not exist and will not be created");
        }
    }
    if (status != ACH_OK)
    {
        ROS_FATAL("Unable to open ACH channel: %s with error: %s (code: %d)", channel_name_.c_str(), ach_result_to_string(status), status);
        throw std::invalid_argument("ACH channel: " + channel_name_ + " is invalid and could not be opened");
    }
    else
    {
        ROS_INFO("Opened ACH channel: %s", channel_name_.c_str());
    }
}

template <typename T>
ACH_ROS_WRAPPER<T>::~ACH_ROS_WRAPPER()
{
    ACH_ROS_WRAPPER::CancelOperations();
    ACH_ROS_WRAPPER::CloseChannel();
}

template <typename T>
const bool ACH_ROS_WRAPPER<T>::CancelOperations()
{
    // Waiting for this functionality
    return false;
}

template <typename T>
const bool ACH_ROS_WRAPPER<T>::CloseChannel(bool destroy_channel)
{
    ach_status_t status = ACH_OK;
    status = ach_close(&ach_channel_);
    if (destroy_channel)
    {
        status = ach_unlink(channel_name_.c_str());
    }
    return true;
}

template <typename T>
const bool ACH_ROS_WRAPPER<T>::WriteState(const T &data)
{
    ach_data_ = data;
    ach_status_t status = ACH_OK;
    status = ach_put(&ach_channel_, &ach_data_, sizeof(ach_data_));
    if (status == ACH_OVERFLOW)
    {
        ROS_ERROR("Overflow writing data to ACH channel: %s", channel_name_.c_str());
        throw std::string("Overflow writing data to ACH channel: ") + channel_name_;
    }
    else if (status != ACH_OK)
    {
        ROS_ERROR("Unable to write data to ACH channel: %s with error: %s (code: %d)", channel_name_.c_str(), ach_result_to_string(status), status);
        throw std::string("Unable to write data to ACH channel: ") + channel_name_;
    }
    else
    {
        return true;
    }
}

template <typename T>
T ACH_ROS_WRAPPER<T>::ReadNextState()
{
    ach_status_t status = ACH_OK;
    size_t fs = 0;
    status = ach_get(&ach_channel_, &ach_data_, sizeof(ach_data_), &fs, NULL, ACH_O_WAIT);
    if (status == ACH_STALE_FRAMES)
    {
        ROS_WARN("No new data on ACH channel: %s", channel_name_.c_str());
        throw std::string("No new data on ACH channel: ") + channel_name_;
    }
    else if (status == ACH_MISSED_FRAME)
    {
        ROS_WARN("Missed frame on ACH channel: %s", channel_name_.c_str());
    }
    else if (status != ACH_OK)
    {
        ROS_ERROR("Problem reading from ACH channel: %s with error: %s (code: %d)", channel_name_.c_str(), ach_result_to_string(status), status);
        throw std::string("Problem reading from ACH channel: ") + channel_name_;
    }
    if (fs != sizeof(ach_data_))
    {
        ROS_ERROR("Data received over ACH channel: %s is not the expected size", channel_name_.c_str());
        throw std::length_error("Data received over ACH channel: " + channel_name_ + "is not the expected size");
    }
    return ach_data_;
}

template <typename T>
T ACH_ROS_WRAPPER<T>::ReadNextState(double wait_seconds)
{
    timespec wait_time;
    wait_time.tv_sec = (int64_t)wait_seconds;
    wait_time.tv_nsec = (int64_t)((wait_seconds - wait_time.tv_sec) * 1000000000.0);
    return ACH_ROS_WRAPPER::ReadNextState(wait_time);
}

template <typename T>
T ACH_ROS_WRAPPER<T>::ReadNextState(timespec wait_time)
{
    ach_status_t status = ACH_OK;
    size_t fs = 0;
    timespec base_time;
    if (clock_gettime(ACH_DEFAULT_CLOCK, &base_time) != 0)
    {
        ROS_ERROR("Unable to get current ACH system time");
        throw std::string("Unable to get current ACH system time");
    }
    int64_t nsecs = wait_time.tv_nsec + base_time.tv_nsec;
    int64_t secs = (wait_time.tv_sec + base_time.tv_sec) + (nsecs / 1000000000);
    nsecs = nsecs % 1000000000;
    wait_time.tv_nsec = nsecs;
    wait_time.tv_sec = secs;
    status = ach_get(&ach_channel_, &ach_data_, sizeof(ach_data_), &fs, &wait_time, ACH_O_WAIT);
    if (status == ACH_TIMEOUT)
    {
        ROS_WARN("ACH channel: %s timed out waiting for data", channel_name_.c_str());
        throw std::string("ACH channel: " + channel_name_ + " timed out waiting to read");
    }
    else if (status == ACH_STALE_FRAMES)
    {
        ROS_WARN("No new data on ACH channel: %s", channel_name_.c_str());
        throw std::string("No new data on ACH channel: ") + channel_name_;
    }
    else if (status == ACH_MISSED_FRAME)
    {
        ROS_WARN("Missed frame on ACH channel: %s", channel_name_.c_str());
    }
    else if (status != ACH_OK)
    {
        ROS_ERROR("Problem reading from ACH channel: %s with error: %s (code: %d)", channel_name_.c_str(), ach_result_to_string(status), status);
        throw std::string("Problem reading from ACH channel: ") + channel_name_;
    }
    if (fs != sizeof(ach_data_))
    {
        ROS_ERROR("Data received over ACH channel: %s is not the expected size", channel_name_.c_str());
        throw std::length_error("Data received over ACH channel: " + channel_name_ + "is not the expected size");
    }
    return ach_data_;
}

template <typename T>
T ACH_ROS_WRAPPER<T>::ReadLastState()
{
    ach_status_t status = ACH_OK;
    size_t fs = 0;
    status = ach_get(&ach_channel_, &ach_data_, sizeof(ach_data_), &fs, NULL, ACH_O_LAST);
    if (status == ACH_STALE_FRAMES)
    {
        ROS_WARN("No new data on ACH channel: %s", channel_name_.c_str());
        throw std::string("No new data on ACH channel: ") + channel_name_;
    }
    else if (status == ACH_MISSED_FRAME)
    {
        ROS_WARN("Missed frame on ACH channel: %s", channel_name_.c_str());
    }
    else if (status != ACH_OK)
    {
        ROS_ERROR("Problem reading from ACH channel: %s", channel_name_.c_str());
        throw std::string("Problem reading from ACH channel: ") + channel_name_;
    }
    if (fs != sizeof(ach_data_))
    {
        ROS_ERROR("Data received over ACH channel: %s is not the expected size", channel_name_.c_str());
        throw std::length_error("Data received over ACH channel: " + channel_name_ + "is not the expected size");
    }
    return ach_data_;
}

#endif // ACH_ROS_WRAPPER_H
