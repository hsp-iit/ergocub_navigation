#include "PointcloudFilter/PointcloudFilter.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/ModelCoefficients.h"

#include <cmath>
#include <chrono>

using std::placeholders::_1;


void PointcloudFilter::depth_callback(const sensor_msgs::msg::PointCloud2::ConstPtr& pc_in)
    {
        try
        {    
            auto tp = std::chrono::system_clock::now();
            double delta_t = (std::chrono::duration<double, std::milli>(tp.time_since_epoch()).count() - std::chrono::duration<double, std::milli>(m_last_vibration_detection.time_since_epoch()).count());
            //wait if a vibration was detected before the timeout
            if (delta_t < m_ms_wait) 
            {
                //RCLCPP_INFO(this->get_logger(), "Exiting scan callback, time passed: %f vs timeout: %f", delta_t, m_ms_wait);
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
            /* //TOO SLOW
            pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*pc_in, *in_cloud);

            // Create the filtering object
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud (in_cloud);
            sor.setMeanK (50);
            sor.setStddevMulThresh (1.0);
            sor.filter (*cloud_filtered);
            sensor_msgs::msg::PointCloud2 pc_out;
            pcl::toROSMsg(*cloud_filtered, pc_out);
            m_filtered_pointcloud_pub->publish(pc_out);
            */
 
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

PointcloudFilter::PointcloudFilter() : Node("depth_filtering_node")
{
    m_filtered_pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(m_pub_topic, 10);
    m_unfiltered_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(m_pub_unfiltered_topic , 10);
    m_last_vibration_detection = std::chrono::system_clock::now();

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
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    if (rclcpp::ok())
    {
        auto node = std::make_shared<PointcloudFilter>();
        std::cout << "Starting up node. \n";
        rclcpp::spin(node);
        std::cout << "Shutting down" << std::endl;
        rclcpp::shutdown();
    }
    else
    {
        std::cout << "ROS2 not available. Shutting down node. \n";
    }
    
    return 0;
}
