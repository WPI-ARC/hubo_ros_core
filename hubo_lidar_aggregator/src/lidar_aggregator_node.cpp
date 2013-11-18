#include <ros/ros.h>
#include <math.h>
#include <string>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <hubo_sensor_msgs/LidarAggregation.h>

// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>

std::string g_fixed_frame;
ros::Publisher g_cloud_publisher;
laser_geometry::LaserProjection g_laser_projector;
tf::TransformListener* g_transformer;

bool LaserAggregationServiceCB(hubo_sensor_msgs::LidarAggregation::Request& req, hubo_sensor_msgs::LidarAggregation::Response& res)
{
    ROS_INFO("Attempting to aggregate %ld laser scans into a pointcloud", req.Scans.size());
    sensor_msgs::PointCloud2 full_cloud;
    if (req.Scans.size() > 1)
    {
        g_laser_projector.transformLaserScanToPointCloud(g_fixed_frame, req.Scans[0], full_cloud, *g_transformer);
        for (unsigned int index = 1; index < req.Scans.size(); index++)
        {
            sensor_msgs::PointCloud2 scan_cloud;
            g_laser_projector.transformLaserScanToPointCloud(g_fixed_frame, req.Scans[index], scan_cloud, *g_transformer);
            bool succeded = pcl::concatenatePointCloud(full_cloud, scan_cloud, full_cloud);
            if (!succeded)
            {
                ROS_ERROR("PCL could not concatenate pointclouds");
            }
        }
    }
    else if (req.Scans.size() == 1)
    {
        g_laser_projector.transformLaserScanToPointCloud(g_fixed_frame, req.Scans[0], full_cloud, *g_transformer);
    }
    else
    {
        ROS_WARN("No laser scans to aggregate");
    }
    full_cloud.header.frame_id = g_fixed_frame;
    full_cloud.header.stamp = ros::Time::now();
    res.Cloud = full_cloud;
    g_cloud_publisher.publish(full_cloud);
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_aggregator");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    tf::TransformListener listener(nh, ros::Duration(20.0));
    g_transformer = &listener;
    ROS_INFO("Starting LIDAR aggregator...");
    nhp.param(std::string("fixed_frame"), g_fixed_frame, std::string("Body_TSY"));
    g_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 1, true);
    ros::ServiceServer server = nh.advertiseService("aggregate_lidar", LaserAggregationServiceCB);
    ROS_INFO("LIDAR aggregator loaded");
    while (ros::ok())
    {
        ros::spinOnce();
    }
    ROS_INFO("Shutting down LIDAR aggregator");
    return 0;
}
