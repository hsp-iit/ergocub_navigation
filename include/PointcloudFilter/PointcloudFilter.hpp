#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "pcl/filters/passthrough.h"
#include "pcl/ModelCoefficients.h"
#include "pcl/filters/project_inliers.h"

#include <mutex>

class PointcloudFilter : public rclcpp::Node
{
private:
    const std::string m_referece_frame = "geometric_unicycle";  //virtual_unicycle_base
    const std::string m_depth_topic = "/camera/depth/color/points";
    const std::string m_pub_topic = "/imu_filtered_depth";
    const std::string m_imu_topic = "/head_imu";

    const double m_close_threshold = 0.5;
    const double m_imuVel_x_threshold = 0.4;
    const double m_imuVel_y_threshold = 0.4;
    const double m_ms_wait = 400.0;
    std::chrono::system_clock::time_point m_last_vibration_detection;

    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer_in;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_filtered_pointcloud_pub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_raw_depth_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;
    geometry_msgs::msg::Vector3 m_imu_angular_velocity;
    std::mutex m_imu_mutex;

    void depth_callback(const sensor_msgs::msg::PointCloud2::ConstPtr& pc_in);

    void imu_callback(const sensor_msgs::msg::Imu::ConstPtr& imu_msg);

public:
    PointcloudFilter();
};  // End of class PointcloudFilter node