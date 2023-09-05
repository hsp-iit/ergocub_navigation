#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <mutex>

class PointcloudFilter : public rclcpp::Node
{
private:
    const std::string m_referece_frame = "geometric_unicycle";  //virtual_unicycle_base
    const std::string m_depth_topic = "/camera/depth/color/points";
    const std::string m_pub_topic = "/imu_filtered_depth";
    const std::string m_pub_unfiltered_topic = "/imu_unfiltered_depth";
    const std::string m_imu_topic = "/head_imu";

    const double m_imuVel_x_threshold = 0.3;
    const double m_imuVel_y_threshold = 0.3;
    const double m_ms_wait = 500.0;
    std::chrono::system_clock::time_point m_last_vibration_detection;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_filtered_pointcloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_unfiltered_pub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_raw_depth_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;
    geometry_msgs::msg::Vector3 m_imu_angular_velocity;
    std::mutex m_imu_mutex;

    void depth_callback(const sensor_msgs::msg::PointCloud2::ConstPtr& pc_in);

    void imu_callback(const sensor_msgs::msg::Imu::ConstPtr& imu_msg);

public:
    PointcloudFilter();
};  // End of class PointcloudFilter node