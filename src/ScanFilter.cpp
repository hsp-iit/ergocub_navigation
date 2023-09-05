#include "ScanFilter/ScanFilter.hpp"

#include <pcl/filters/extract_indices.h>

#include <cmath>
#include <chrono>

using std::placeholders::_1;


void ScanFilter::scan_callback(const sensor_msgs::msg::LaserScan::ConstPtr& scan_in)
    {
        try
        {
        auto tp = std::chrono::system_clock::now();
        double delta_t = (std::chrono::duration<double, std::milli>(tp.time_since_epoch()).count() - std::chrono::duration<double, std::milli>(m_last_vibration_detection.time_since_epoch()).count());
        if (delta_t < m_ms_wait) 
        {
            //RCLCPP_DEBUG(this->get_logger(), "Exiting scan callback, time passed: %f vs timeout: %f", delta_t, m_ms_wait);
            return;
        }
        
        //IMU
        {
            std::lock_guard<std::mutex>lock(m_imu_mutex);
            if (std::abs(m_imu_angular_velocity.x) > m_imuVel_x_threshold || std::abs(m_imu_angular_velocity.y) > m_imuVel_y_threshold)
            {
                //RCLCPP_DEBUG(this->get_logger(), "Imu detected high velocities: x: %f y: %f", m_imu_angular_velocity.x, m_imu_angular_velocity.y);
                m_last_vibration_detection = std::chrono::system_clock::now();
                return;
            }
        }

        std::string transform_error;
        if (m_tf_buffer_in->canTransform(
                scan_in->header.frame_id,
                m_referece_frame,
                tf2_ros::fromMsg(scan_in->header.stamp) + tf2::durationFromSec(scan_in->ranges.size() * scan_in->time_increment),
                tf2::durationFromSec(0.02),  
                & transform_error ))
            {
                // Converts the scans into cartesian space points
                sensor_msgs::msg::PointCloud2 original_cloud;
                sensor_msgs::msg::PointCloud2 transformed_cloud;
                m_projector.projectLaser(*scan_in, original_cloud);
                original_cloud.header.frame_id = scan_in->header.frame_id;
                //transform cloud from lidar frame to virtual_unicycle_base
                try
                {
                    transformed_cloud = m_tf_buffer_in->transform(original_cloud, m_referece_frame, tf2::durationFromSec(0.0));
                }
                catch(const std::exception& e)
                {
                   RCLCPP_WARN(this->get_logger(), "Cannot transform %s to %s: %s \n", original_cloud.header.frame_id, m_referece_frame, e.what());
                   return;
                }
                //PCL Clouds
                pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered1 (new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);

                pcl::fromROSMsg(transformed_cloud, *pcl_cloud);
                // Exlude the points too close to the center for the crane - TODO verify it
                if(m_robot_on_crane)
                {
                    int original_size = pcl_cloud->size();
                    pcl::PointIndices::Ptr close_points(new pcl::PointIndices());
                    pcl::ExtractIndices<pcl::PointXYZ> filter;
                    for(size_t i = 0; i < pcl_cloud->size(); ++i)
                    {
                        if (std::sqrt(std::pow(pcl_cloud->points[i].x, 2) + std::sqrt(std::pow(pcl_cloud->points[i].y, 2))) < m_close_threshold)
                        {
                            close_points->indices.push_back(i);
                        }
                    }
                    filter.setInputCloud(pcl_cloud);
                    filter.setIndices(close_points);
                    // Retrieve indices to all points in pcl_cloud except those referenced by indices close_points
                    filter.setNegative (true);
                    filter.filter(*pcl_cloud);
                    RCLCPP_DEBUG(this->get_logger(), "Removed %f points", pcl_cloud->size() - original_size);
                }

                // Filter cloud
                // FILTER 1 - PASSTHROUGH
                pcl::PassThrough<pcl::PointXYZ> pass;
                pass.setInputCloud (pcl_cloud);
                pass.setFilterFieldName ("z");
                pass.setFilterLimits(m_filter_z_low, m_filter_z_high);
                pass.filter (*cloud_filtered1);

                // FILTER 2 - PLANE PROJECTOR
                pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());   // Create a set of planar coefficients with X=Y=0,Z=1, d= 1.5
                coefficients->values.resize(4);
                coefficients->values[0] = coefficients->values[1] = 0;
                coefficients->values[2] = 1;
                coefficients->values[3] = 0;
                pcl::ProjectInliers<pcl::PointXYZ> proj;
                proj.setModelType (pcl::SACMODEL_PLANE);
                proj.setInputCloud (cloud_filtered1);
                proj.setModelCoefficients (coefficients);
                proj.filter (*cloud_filtered2);

                //Publish
                sensor_msgs::msg::PointCloud2 ros_cloud;
                ros_cloud.header.frame_id = m_referece_frame;
                ros_cloud.header.stamp = scan_in->header.stamp;
                pcl::toROSMsg(*cloud_filtered2, ros_cloud);
                m_pointcloud_pub->publish(ros_cloud);
            }
            else
            {
                RCLCPP_ERROR(get_logger(), "Could not transform message: %s", transform_error.c_str());
            }
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(get_logger(), e.what());
        }
    };

void ScanFilter::imuCallback(const sensor_msgs::msg::Imu::ConstPtr& imu_msg)
{
    try
    {
        std::lock_guard<std::mutex>lock(m_imu_mutex);
        m_imu_angular_velocity = imu_msg->angular_velocity;
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(get_logger(), e.what());
    }
};

ScanFilter::ScanFilter() : Node("scan_compensating_node")
{
    m_pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(m_pub_topic, 10);
    m_tf_buffer_in = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer_in);
    m_last_vibration_detection = std::chrono::system_clock::now();

    m_raw_scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan> (
        m_scan_topic,
        10,
        std::bind(&ScanFilter::scan_callback, this, _1)
    );
    m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu> (
        m_imu_topic,
        10,
        std::bind(&ScanFilter::imuCallback, this, _1)
    );
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    if (rclcpp::ok())
    {
        auto node = std::make_shared<ScanFilter>();
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
