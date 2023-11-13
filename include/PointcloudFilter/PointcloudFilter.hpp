#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <mutex>

class PointcloudFilter : public rclcpp_lifecycle::LifecycleNode
{
private:
    //Parameters
    std::string m_depth_topic = "/camera/depth/color/points";
    std::string m_pub_topic = "/imu_filtered_depth";
    std::string m_pub_unfiltered_topic = "/imu_unfiltered_depth";
    std::string m_imu_topic = "/head_imu";
    std::string m_filter_reference_frame;  //reference frame in the center of the box filter

    double m_box_x, m_box_y, m_box_z, m_box_w;   //vertex extremes for box filter
    double m_imuVel_x_threshold = 0.3;
    double m_imuVel_y_threshold = 0.3;
    double m_ms_wait = 500.0;
    std::chrono::system_clock::time_point m_last_vibration_detection;
    bool m_extract_removed_indices;
    bool m_set_negative;

    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;

    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_filtered_pointcloud_pub;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_unfiltered_pub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_raw_depth_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;
    geometry_msgs::msg::Vector3 m_imu_angular_velocity;
    std::mutex m_imu_mutex;

    void depth_callback(const sensor_msgs::msg::PointCloud2::ConstPtr& pc_in);

    void imu_callback(const sensor_msgs::msg::Imu::ConstPtr& imu_msg);

public:
    PointcloudFilter(const rclcpp::NodeOptions &options);

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturn on_configure(const rclcpp_lifecycle::State &);
    CallbackReturn on_activate(const rclcpp_lifecycle::State &);
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);
    CallbackReturn on_error(const rclcpp_lifecycle::State & state);
};  // End of class PointcloudFilter node