/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SCAN_FILTER__HPP
#define SCAN_FILTER__HPP

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

class PlaneDetector : public rclcpp_lifecycle::LifecycleNode
{
private:
    // Parameter List
    std::string m_head_frame = "realsense";    // reference frame to which all the computations are performed
    std::string m_contact_frame = "r_sole";             // reference frame of the foot in contact with the ground
    std::string m_pc_topic = "/camera/depth/color/points";               // topic name of the lidar scan
    std::string m_pub_topic = "/adjusted_depth_pc";           // name of the topic where the filtered pointcloud will be published

    float m_filter_z_low = 0.2;             // minimum height to accept laser readings, from the reference frame, in meters
    float m_filter_z_high = 2.5;            // maximum height to accept laser readings, from the reference frame, in meters

    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer_in;

    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pointcloud_pub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_pc_sub;

    void pc_callback(const sensor_msgs::msg::PointCloud2::ConstPtr& pc_in);

public:
    PlaneDetector(const rclcpp::NodeOptions & options);

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturn on_configure(const rclcpp_lifecycle::State &);
    CallbackReturn on_activate(const rclcpp_lifecycle::State &);
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);
    CallbackReturn on_error(const rclcpp_lifecycle::State & state);
};  // End of class PlaneDetector node

#endif

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>


int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    
    // Optional
    seg.setOptimizeCoefficients (true);
    
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    
    if (inliers->indices.size () == 0)
    {
      PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
      return (-1);
    }
    
    std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                        << coefficients->values[1] << " "
                                        << coefficients->values[2] << " " 
                                        << coefficients->values[3] << std::endl;
    
    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
    
    for (const auto& idx: inliers->indices)
        std::cerr << idx << "    " << cloud->points[idx].x << " "
                                   << cloud->points[idx].y << " "
                                   << cloud->points[idx].z << std::endl;
    
    return (0);
}