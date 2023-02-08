#pragma once
#include "std_msgs/String.h"

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "laser_geometry/laser_geometry.h"
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


float min_ang_, max_ang_, range_min_, range_max_;
std::string frame_id_;
ros::Publisher scan_pub_, cloud_pub_;
sensor_msgs::PointCloud2 cloud_1;
sensor_msgs::PointCloud2 cloud_2;
sensor_msgs::PointCloud2 cloud_3;
sensor_msgs::PointCloud2 concatenated_cloud;
sensor_msgs::LaserScan concatenated_scan;
laser_geometry::LaserProjection projector_;

void chatterCallback_4(const sensor_msgs::LaserScan &msg)
{
}

void chatterCallback_8(const sensor_msgs::LaserScan &msg)
{
}
sensor_msgs::LaserScanPtr pointcloud_to_laserscan(sensor_msgs::PointCloud2 *merged_cloud)
{
    sensor_msgs::LaserScanPtr output(new sensor_msgs::LaserScan());
    output->header = merged_cloud->header;
    output->header.frame_id = "base_link";
    output->header.stamp = ros::Time::now();
    output->angle_min = -3.12414;
    // -179 degrees
    output->angle_max = 3.12414;
    // +179 degrees
    output->angle_increment = 0.0174532923847;
    output->time_increment = 0.000185185184819;
    output->scan_time = 0.0666666701436;
    output->range_min = range_min_;
    output->range_max = range_max_;
    float inf = std::numeric_limits<float>::infinity();
    uint32_t ranges_size = std::ceil((output->angle_max - output->angle_min) / output->angle_increment);
    output->ranges.assign(ranges_size, inf);
    output->intensities.assign(ranges_size, 0);
    for (sensor_msgs::PointCloud2ConstIterator<float> it(*merged_cloud, "x"); it != it.end(); ++it)
    {
        const float &x = it[0];
        const float &y = it[1];
        const float &z = it[2];
        const float &intensity = it[3];
        if (std::isnan(x) || std::isnan(y) || std::isnan(z))
        {
            ROS_DEBUG("rejected for nan in point(%f, %f, %f)\n", x, y, z);
            continue;
        }

        double range_sq = y * y + x * x;
        double intensity_sq = intensity;
        double range_min_sq_ = output->range_min * output->range_min;
        if (range_sq < range_min_sq_)
        {
            ROS_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range_sq, range_min_sq_, x, y, z);
            continue;
        }

        double angle = atan2(y, x);
        if (angle < output->angle_min || angle > output->angle_max)
        {
            ROS_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output->angle_min, output->angle_max);
            continue;
        }
        int index = (angle - output->angle_min) / output->angle_increment;

        if (output->ranges[index] * output->ranges[index] > range_sq)
        {
            output->ranges[index] = sqrt(range_sq);
            output->intensities[index] = intensity_sq;
        }
    }

    return output;
}

void cb3(const sensor_msgs::PointCloud2ConstPtr& cloud_a, const sensor_msgs::PointCloud2ConstPtr& cloud_b, const sensor_msgs::PointCloud2ConstPtr& cloud_c, std::shared_ptr<ros::Publisher> const& pub, std::shared_ptr<tf::TransformListener> const& tfl)
{
    // Get Frame ID
    std::string targetFrame = cloud_a->header.frame_id;

    // Clouds for concatinating
    sensor_msgs::PointCloud2 cloud_concat;
    sensor_msgs::PointCloud2 trans_cloud_b;
    sensor_msgs::PointCloud2 trans_cloud_c;

    // transform cloud_b
    if( !pcl_ros::transformPointCloud(targetFrame, *cloud_b, trans_cloud_b, *tfl) ){
        // Failed to transform
        ROS_WARN("Could not transform 'cloud_b' to 'cloud_a'\n");
    }

    // transform cloud_c
    if( !pcl_ros::transformPointCloud(targetFrame, *cloud_c, trans_cloud_c, *tfl) ){
        // Failed to transform
        ROS_WARN("Could not transform 'cloud_c' to 'cloud_a'\n");
    }
    
    // Concat the pointclouds and build final pointcloud
    pcl::concatenatePointCloud(*cloud_a, trans_cloud_b, cloud_concat);
    pcl::concatenatePointCloud(cloud_concat, trans_cloud_c, cloud_concat);
    cloud_concat.header.stamp = ros::Time::now();
    cloud_concat.header.frame_id = targetFrame;

    // Ship it
    pub -> publish(cloud_concat);
}

void concat_with_pc(std::string& pc1, std::string& pc2, std::string& pc3, ros::NodeHandle& n)
{
    message_filters::Subscriber<sensor_msgs::PointCloud2> scan1_sub_, scan2_sub_, scan3_sub_;

    // Setup sync policies
        //typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> ApproxPolicy2;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> ApproxPolicy3;
        //typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> ApproxPolicy4;
        //typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> ApproxPolicy5;

        // Setup synchronizers
        //typedef message_filters::Synchronizer<ApproxPolicy2> ApproxSync2;
    typedef message_filters::Synchronizer<ApproxPolicy3> ApproxSync3;
        //typedef message_filters::Synchronizer<ApproxPolicy4> ApproxSync4;
        //typedef message_filters::Synchronizer<ApproxPolicy5> ApproxSync5;

    // Setup synchronizer points
        //boost::shared_ptr<ApproxSync2> approx_sync2_;
    boost::shared_ptr<ApproxSync3> approx_sync3_;
        //boost::shared_ptr<ApproxSync4> approx_sync4_;
        //boost::shared_ptr<ApproxSync5> approx_sync5_;

    // Setup Queue_size_
    int queue_size_ = 10;

    
    // TF listener
    std::shared_ptr<tf::TransformListener> tf_listener(new tf::TransformListener());

    //publisher shared pointer
    std::shared_ptr<ros::Publisher> pubPtr(&cloud_pub_);
    while (ros::ok())
    {
       

        boost::shared_ptr<sensor_msgs::PointCloud2 const> sharedPtr;
        sensor_msgs::LaserScan scan_tim_4;
        sharedPtr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(pc1, ros::Duration(1));
        if (sharedPtr == NULL)
            ROS_INFO("No laser messages received pc1.\n");
        else
            cloud_1 = *sharedPtr;
        sensor_msgs::LaserScan scan_tim_8;

        sharedPtr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(pc2, ros::Duration(1));
        if (sharedPtr == NULL)
            ROS_INFO("No laser messages received pc2.\n");
        else
            cloud_2 = *sharedPtr;

        sharedPtr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(pc3, ros::Duration(1));
        if (sharedPtr == NULL)
            ROS_INFO("No laser messages received pc3.\n");
        else
            cloud_3 = *sharedPtr;


        std::string targetFrame = cloud_1.header.frame_id;

        // transform cloud_2
        //const sensor_msgs::PointCloud2ConstPtr cloud_b = &cloud_2;
        scan1_sub_.subscribe(n, pc1, queue_size_);
        scan2_sub_.subscribe(n, pc2, queue_size_);
        scan3_sub_.subscribe(n, pc3, queue_size_);

        approx_sync3_.reset(new ApproxSync3(ApproxPolicy3(queue_size_), scan1_sub_, scan2_sub_, scan3_sub_));
        approx_sync3_->registerCallback(boost::bind(&cb3, _1, _2, _3, pubPtr, tf_listener));

        // pcl::concatenatePointCloud(cloud_1, cloud_2, concatenated_cloud);
        // concatenated_cloud.fields[3].name = "intensity";
        // concatenated_cloud.header.frame_id = "base_link";
        // cloud_pub_.publish(concatenated_cloud);

        // concatenated_scan = *pointcloud_to_laserscan(&concatenated_cloud);
        // scan_pub_.publish(concatenated_scan);

        ros::spin();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pc_fusion");
    ros::NodeHandle n;
    scan_pub_ = n.advertise<sensor_msgs::LaserScan>("concatenated_scan", 1);
    cloud_pub_ = n.advertise<sensor_msgs::PointCloud2>("concatenated_cloud", 1);
    ros::Duration duration = ros::Duration(1.0 / 20.0);
    ros::Rate loop_r(duration);

    // TF listener
    //std::shared_ptr<tf::TransformListener> tf_listener(new tf::TransformListener());


    n.getParam("min_ang", min_ang_);
    n.getParam("max_ang", max_ang_);
    n.getParam("range_min", range_min_);
    n.getParam("range_max", range_max_);
    n.getParam("frame_id", frame_id_);

    std::string pc1 = "/ouster_3/os_cloud_node/points";
    std::string pc2 = "/ouster_4/os_cloud_node/points";
    std::string pc3 = "/ouster_5/os_cloud_node/points";
    concat_with_pc(pc1, pc2, pc3, n);
    

}
