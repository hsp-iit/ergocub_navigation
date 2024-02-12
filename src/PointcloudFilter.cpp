/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "PointcloudFilter/PointcloudFilter.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "pcl/filters/crop_box.h"
#include "pcl/ModelCoefficients.h"
#include <pcl/filters/extract_indices.h>
#include "pcl_ros/transforms.hpp"

#include <cmath>
#include <chrono>

using std::placeholders::_1;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

void PointcloudFilter::depth_callback(const sensor_msgs::msg::PointCloud2::ConstPtr& pc_in)
    {
        try
        {    
            auto tp = std::chrono::system_clock::now();
            double delta_t = (std::chrono::duration<double, std::milli>(tp.time_since_epoch()).count() - std::chrono::duration<double, std::milli>(m_last_vibration_detection.time_since_epoch()).count());
            //wait if a vibration was detected before the timeout
            if (delta_t < m_ms_wait) 
            {
                RCLCPP_INFO(this->get_logger(), "Exiting scan callback, time passed: %f vs timeout: %f", delta_t, m_ms_wait);
                return;
            }

            //IMU
            {
                std::lock_guard<std::mutex>lock(m_imu_mutex);
                if (std::abs(m_imu_angular_velocity.x) > m_imuVel_x_threshold || std::abs(m_imu_angular_velocity.y) > m_imuVel_y_threshold)
                {
                    RCLCPP_INFO(this->get_logger(), "Imu detected high velocities: x: %f y: %f", m_imu_angular_velocity.x, m_imu_angular_velocity.y);
                    m_last_vibration_detection = std::chrono::system_clock::now();
                    return;
                }
            }
            //TOO SLOW
            pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*pc_in, *in_cloud);
            //Transform cloud into filter's reference frame
            try
            {
                auto tf=m_tf_buffer->lookupTransform(in_cloud->header.frame_id, m_filter_reference_frame, now());
                if(! pcl_ros::transformPointCloud(m_filter_reference_frame, *in_cloud, *in_cloud, *m_tf_buffer))
                {
                    std::cout << "Unable to transform pcl_ros" << std::endl;
                    return;
                }
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
                return;
            }
            // Create the filtering object
            pcl::CropBox<pcl::PointXYZ> cropBoxFilter (m_extract_removed_indices);
            cropBoxFilter.setInputCloud (in_cloud);
            Eigen::Vector4f min_pt (-m_box_x, -m_box_y, -m_box_z, m_box_w);
            Eigen::Vector4f max_pt (m_box_x, m_box_y, m_box_z, m_box_w);

            // Cropbox slighlty bigger then bounding box of points
            cropBoxFilter.setMin (min_pt);
            cropBoxFilter.setMax (max_pt);
            cropBoxFilter.setNegative(m_set_negative);

            // Cloud
            pcl::PointCloud<pcl::PointXYZ> cloud_out;
            cropBoxFilter.filter(cloud_out);
            
            sensor_msgs::msg::PointCloud2 pc_out;
            pcl::toROSMsg(cloud_out, pc_out);
            m_filtered_pointcloud_pub->publish(pc_out);
            
 
            //Echo the message
            sensor_msgs::msg::PointCloud2 pc_out_unfiltered;
            pc_out_unfiltered.header = pc_in->header;
            pc_out_unfiltered.data = pc_in->data;
            pc_out_unfiltered.fields = pc_in->fields;
            pc_out_unfiltered.height = pc_in->height;
            pc_out_unfiltered.is_bigendian = pc_in->is_bigendian;
            pc_out_unfiltered.is_dense = pc_in->is_dense;
            pc_out_unfiltered.point_step = pc_in->point_step;
            pc_out_unfiltered.row_step = pc_in->row_step;
            pc_out_unfiltered.width = pc_in->width;
            m_unfiltered_pub->publish(pc_out_unfiltered);
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), e.what());
        }
    };

void PointcloudFilter::imu_callback(const sensor_msgs::msg::Imu::ConstPtr& imu_msg)
{
    try
    {
        std::lock_guard<std::mutex>lock(m_imu_mutex);
        m_imu_angular_velocity = imu_msg->angular_velocity;
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), e.what());
    }
};

PointcloudFilter::PointcloudFilter(const rclcpp::NodeOptions & options) : rclcpp_lifecycle::LifecycleNode("depth_filtering_node", options)
{
    //Parameters declaration
    declare_parameter("depth_topic", "/camera/depth/color/points");
    declare_parameter("pub_topic", "/imu_filtered_depth");
    declare_parameter("pub_unfiltered_topic", "/imu_unfiltered_depth");
    declare_parameter("imu_topic", "/head_imu");
    declare_parameter("filter_reference_frame", "chest");
    declare_parameter("box_x", 0.5);
    declare_parameter("box_y", 0.4);
    declare_parameter("box_z", 0.5);
    declare_parameter("box_w", 1.0);
    declare_parameter("imuVel_x_threshold", 0.3);
    declare_parameter("imuVel_y_threshold", 0.3);
    declare_parameter("ms_wait", 500.0);
    declare_parameter("extract_removed_indices", true);
    declare_parameter("set_negative", true);

    m_last_vibration_detection = std::chrono::system_clock::now();
    m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
}

CallbackReturn PointcloudFilter::on_configure(const rclcpp_lifecycle::State &)
{
    //Pass parameters
    m_depth_topic = this->get_parameter("depth_topic").as_string();
    m_pub_topic = this->get_parameter("pub_topic").as_string();
    m_pub_unfiltered_topic = this->get_parameter("pub_unfiltered_topic").as_string();
    m_imu_topic = this->get_parameter("imu_topic").as_string();
    m_filter_reference_frame = this->get_parameter("filter_reference_frame").as_string();
    m_box_x = this->get_parameter("box_x").as_double();
    m_box_y = this->get_parameter("box_y").as_double();
    m_box_z = this->get_parameter("box_z").as_double();
    m_box_w = this->get_parameter("box_w").as_double();
    m_imuVel_x_threshold = this->get_parameter("imuVel_x_threshold").as_double();
    m_imuVel_y_threshold = this->get_parameter("imuVel_y_threshold").as_double();
    m_ms_wait = this->get_parameter("ms_wait").as_double();
    m_extract_removed_indices = this->get_parameter("extract_removed_indices").as_bool();
    m_set_negative = this->get_parameter("set_negative").as_bool();

    RCLCPP_INFO(get_logger(), "Configuring with: depth_topic: %s pub_topic: %s pub_unfiltered_topic: %s imu_topic: %s filter_reference_frame: %s",
                    m_depth_topic, m_pub_topic, m_pub_unfiltered_topic, m_imu_topic, m_filter_reference_frame);
    RCLCPP_INFO(get_logger(), "box_x: %f box_y: %f box_z: %f box_w: %f imuVel_x_threshold: %f imuVel_y_threshold: %f ms_wait: %f extract_removed_indices: %i set_negative: %i",
                    m_box_x, m_box_y, m_box_z, m_box_w, m_imuVel_x_threshold, m_imuVel_y_threshold, m_ms_wait, (int)m_extract_removed_indices, (int)m_set_negative);

    //Subscribers
    m_raw_depth_sub = this->create_subscription<sensor_msgs::msg::PointCloud2> (
        m_depth_topic,
        10,
        std::bind(&PointcloudFilter::depth_callback, this, _1)
    );
    m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu> (
        m_imu_topic,
        10,
        std::bind(&PointcloudFilter::imu_callback, this, _1)
    );

    //Publishers
    m_filtered_pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(m_pub_topic, 10);
    m_unfiltered_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(m_pub_unfiltered_topic , 10);

    return CallbackReturn::SUCCESS;
}

CallbackReturn PointcloudFilter::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Activating");
    m_filtered_pointcloud_pub->on_activate();
    m_unfiltered_pub->on_activate();
    return CallbackReturn::SUCCESS;
}

CallbackReturn PointcloudFilter::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Deactivating");
    m_filtered_pointcloud_pub->on_deactivate();
    m_unfiltered_pub->on_deactivate();
    return CallbackReturn::SUCCESS;
}

CallbackReturn PointcloudFilter::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Cleaning up");
    m_filtered_pointcloud_pub.reset();
    m_unfiltered_pub.reset();
    m_raw_depth_sub.reset();
    m_imu_sub.reset();
    return CallbackReturn::SUCCESS;
}

CallbackReturn PointcloudFilter::on_shutdown(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "Shutting Down from %s", state.label().c_str());
    return CallbackReturn::SUCCESS;
}

CallbackReturn PointcloudFilter::on_error(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "Error Processing from %s", state.label().c_str());
    return CallbackReturn::SUCCESS;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    if (rclcpp::ok())
    {
        rclcpp::executors::SingleThreadedExecutor executor;
        rclcpp::NodeOptions options;
        auto node = std::make_shared<PointcloudFilter>(options);
        executor.add_node(node->get_node_base_interface());
        std::cout << "Starting up node. \n";
        executor.spin();
        std::cout << "Shutting down" << std::endl;
        rclcpp::shutdown();
    }
    else
    {
        std::cout << "ROS2 not available. Shutting down node. \n";
    }
    
    return 0;
}
