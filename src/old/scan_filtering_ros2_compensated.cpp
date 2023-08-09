#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "laser_geometry/laser_geometry.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "pcl/filters/passthrough.h"
#include "pcl/ModelCoefficients.h"
#include "pcl/filters/project_inliers.h"

#include <mutex>
#include <cmath>
#include <chrono>

using std::placeholders::_1;

class ScanFilter : public rclcpp::Node
{
private:
    const std::string m_referece_frame = "geometric_unicycle";  //virtual_unicycle_base
    const char* m_scan_topic = "/scan_local";
    const std::string m_pub_topic = "/compensated_pc2";
    const std::string m_imu_topic = "/head_imu";
    laser_geometry::LaserProjection m_projector;

    const float m_filter_z_low = 0.2;
    const float m_filter_z_high = 3.0;
    const float m_sensor_height = 1.5;
    const double m_close_threshold = 0.5;
    const double m_imuVel_x_threshold = 0.4;
    const double m_imuVel_y_threshold = 0.4;
    const double m_ms_wait = 500.0;
    std::chrono::system_clock::time_point m_last_vibration_detection;

    const char* m_right_sole_frame = "r_sole";
    const char* m_left_sole_frame = "l_sole";

    bool m_robot_on_crane = true;

    geometry_msgs::msg::TransformStamped m_compensation_TF;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer_in;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pointcloud_pub, m_second_pub, m_third_pub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_raw_scan_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;
    geometry_msgs::msg::Vector3 m_imu_angular_velocity;
    std::mutex m_imu_mutex;

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstPtr& scan_in)
    {
        auto tp = std::chrono::system_clock::now();
        double delta_t = (std::chrono::duration<double, std::milli>(tp.time_since_epoch()).count() - std::chrono::duration<double, std::milli>(m_last_vibration_detection.time_since_epoch()).count());
        RCLCPP_INFO(this->get_logger(), "time passed: %f", delta_t);
        if (delta_t < m_ms_wait) 
        {
            RCLCPP_INFO(this->get_logger(), "Exiting scan callback, time passed: %f vs timeout: %f", delta_t, m_ms_wait);
            return;
        }
        
        //IMU -> should be put first and return before computing laser scan stuff
        {
            std::lock_guard<std::mutex>lock(m_imu_mutex);
            if (std::abs(m_imu_angular_velocity.x) > m_imuVel_x_threshold || std::abs(m_imu_angular_velocity.y) > m_imuVel_y_threshold)
            {
                RCLCPP_INFO(this->get_logger(), "Imu detected high velocities: x: %f y: %f", m_imu_angular_velocity.x, m_imu_angular_velocity.y);
                m_last_vibration_detection = std::chrono::system_clock::now();
                return;
            }
        }
        std::string transform_error;
        if (m_tf_buffer_in->canTransform(
            scan_in->header.frame_id,
            m_referece_frame,
            tf2_ros::fromMsg(scan_in->header.stamp) + tf2::durationFromSec(scan_in->ranges.size() * scan_in->time_increment),
            tf2::durationFromSec(0.2),  
            & transform_error ))
        {
            // Converts the scans into cartesian space points
            sensor_msgs::msg::PointCloud2 original_cloud;
            sensor_msgs::msg::PointCloud2 transformed_cloud;
            m_projector.projectLaser(*scan_in, original_cloud);
            //transform cloud from lidar frame to virtual_unicycle_base
            try
            {
                transformed_cloud = m_tf_buffer_in->transform(original_cloud, m_referece_frame, tf2::durationFromSec((0)));
            }
            catch(const std::exception& e)
            {
               RCLCPP_ERROR(this->get_logger(), "Cannot transform %s from lidar frame to virtual_unicycle_base: %s \n", original_cloud.header.frame_id, e.what());
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
                for(auto it=pcl_cloud->begin(); it==pcl_cloud->end(); ++it)
                {
                    if (std::sqrt(std::pow(it->x, 2) + std::sqrt(std::pow(it->y, 2))) < m_close_threshold)
                    {
                        pcl_cloud->erase(it);
                    }
                }
            }
            
            // Filter cloud
            //RCLCPP_INFO(get_logger(), "Original cloud size: %li \n", pcl_cloud->size());
            // FILTER 1 - PASSTHROUGH
            pcl::PassThrough<pcl::PointXYZ> pass;       // Create the passthrough filtering object
            pass.setInputCloud (pcl_cloud);
            pass.setFilterFieldName ("z");
            pass.setFilterLimits(m_filter_z_low, m_filter_z_high);
            // Filtering
            pass.filter (*cloud_filtered1);
            
            // FILTER 2 - PLANE PROJECTOR
            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());   // Create a set of planar coefficients with X=Y=0,Z=1, d= 1.5
            coefficients->values.resize(4);
            coefficients->values[0] = coefficients->values[1] = 0;
            coefficients->values[2] = 1;
            coefficients->values[3] = 0;
            // Create the filtering object
            pcl::ProjectInliers<pcl::PointXYZ> proj;
            proj.setModelType (pcl::SACMODEL_PLANE);
            proj.setInputCloud (cloud_filtered1);
            proj.setModelCoefficients (coefficients);
            proj.filter (*cloud_filtered2);
            //RCLCPP_INFO(get_logger(), "Cloud size after filter 2: %li \n", cloud_filtered2->size());
            // Publish PC2
            sensor_msgs::msg::PointCloud2 ros_cloud;
            ros_cloud.header.frame_id = m_referece_frame;
            ros_cloud.header.stamp = scan_in->header.stamp;
            pcl::toROSMsg(*cloud_filtered2, ros_cloud);
            m_pointcloud_pub->publish(ros_cloud);
            m_second_pub->publish(ros_cloud);
            m_third_pub->publish(ros_cloud);
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Could not transform message: %s", transform_error.c_str());
        }
    };

    void imuCallback(const sensor_msgs::msg::Imu::ConstPtr& imu_msg)
    {
        std::lock_guard<std::mutex>lock(m_imu_mutex);
        //RCLCPP_INFO(this->get_logger(), "Got imu message with vel x: %f y: %f z: %f", imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
        m_imu_angular_velocity = imu_msg->angular_velocity;
    };

public:
    ScanFilter() : Node("scan_compensating_node")
    {
        m_pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(m_pub_topic, 10);
        m_second_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(m_pub_topic + "_2", 10);
        m_third_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(m_pub_topic + "_3", 10);
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
};  // End of class ScanFilter node


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

