#include "PlaneDetector/PlaneDetector.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>


using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using std::placeholders::_1;

void PlaneDetector::pc_callback(const sensor_msgs::msg::PointCloud2::ConstPtr& pc_in)
{
    if (!(m_left_foot_contact && m_right_foot_contact))
    {
        return;
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pc_in, *in_cloud);

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
    
    seg.setInputCloud (in_cloud);
    seg.segment (*inliers, *coefficients);
    
    if (inliers->indices.size () == 0)
    {
        RCLCPP_ERROR(get_logger(), "Could not estimate a planar model for the given cloud.");
        return;
    }
    
    std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                        << coefficients->values[1] << " "
                                        << coefficients->values[2] << " " 
                                        << coefficients->values[3] << std::endl;
    
    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
    
    for (const auto& idx: inliers->indices)
        std::cerr << idx << "    " << in_cloud->points[idx].x << " "
                                   << in_cloud->points[idx].y << " "
                                   << in_cloud->points[idx].z << std::endl;
}

PlaneDetector::PlaneDetector(const rclcpp::NodeOptions & options) : rclcpp_lifecycle::LifecycleNode("floor_detector_node", options)
{
    declare_parameter("head_frame", "realsense");
    declare_parameter("contact_frame", "r_sole");
    declare_parameter("pointcloud_topic", "/camera/depth/color/points");
    declare_parameter("pub_topic", "/adjusted_depth_pc");

    
    m_tf_buffer_in = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer_in);
}

CallbackReturn PlaneDetector::on_configure(const rclcpp_lifecycle::State &)
{
    //Param Init
    m_head_frame = this->get_parameter("head_frame").as_string();
    m_contact_frame = this->get_parameter("contact_frame").as_string();
    m_pc_topic = this->get_parameter("pointcloud_topic").as_string();
    m_pub_topic = this->get_parameter("pub_topic").as_string();

    m_filter_z_low = this->get_parameter("filter_z_low").as_double();
    m_filter_z_high = this->get_parameter("filter_z_high").as_double();

    RCLCPP_INFO(this->get_logger(), "Configuring with: head_frame: %s contact_frame: %s pointcloud_topic: %s pub_topic: %s ",
                    m_head_frame.c_str(), m_contact_frame.c_str(), m_pc_topic.c_str(), m_pub_topic.c_str());
    
    //Subscribers
    m_pc_sub = this->create_subscription<sensor_msgs::msg::PointCloud2> (
        m_pc_topic,
        10,
        std::bind(&PlaneDetector::pc_callback, this, _1)
    );

    m_right_foot_sub = this->create_subscription<geometry_msgs::msg::WrenchStamped> (
        m_right_foot_topic,
        10,
        std::bind(&PlaneDetector::right_foot_callback, this, _1)
    );
    m_left_foot_sub = this->create_subscription<geometry_msgs::msg::WrenchStamped> (
        m_left_foot_topic,
        10,
        std::bind(&PlaneDetector::left_foot_callback, this, _1)
    );
    
    //Publisher
    m_pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(m_pub_topic, 10);

    return CallbackReturn::SUCCESS;
}

void PlaneDetector::right_foot_callback(const geometry_msgs::msg::WrenchStamped::ConstPtr &msg)
{
    if (-msg->wrench.force.z >= m_wrench_threshold)  //detecting contact
    {
        if (!m_right_foot_contact)
        {
            m_right_foot_contact = true;
            RCLCPP_INFO(this->get_logger(), "Right Foot in contact");
        }
    }
    else{m_right_foot_contact = false;}
}

void PlaneDetector::left_foot_callback(const geometry_msgs::msg::WrenchStamped::ConstPtr &msg)
{
    if (-msg->wrench.force.z >= m_wrench_threshold)  //detecting contact
    {
        if (!m_left_foot_contact)
        {
            m_left_foot_contact = true;
            RCLCPP_INFO(this->get_logger(), "Left Foot in contact");
        }
    }
    else{m_left_foot_contact = false;}
}

CallbackReturn PlaneDetector::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Activating");
    m_pointcloud_pub->on_activate();
    return CallbackReturn::SUCCESS;
}

CallbackReturn PlaneDetector::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Deactivating");
    m_pointcloud_pub->on_deactivate();
    return CallbackReturn::SUCCESS;
}

CallbackReturn PlaneDetector::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Cleaning Up");
    m_pointcloud_pub.reset();
    m_pc_sub.reset();
    return CallbackReturn::SUCCESS;
}

CallbackReturn PlaneDetector::on_shutdown(const rclcpp_lifecycle::State & state)
{
    RCLCPP_INFO(get_logger(), "Shutting Down from %s", state.label().c_str());
    return CallbackReturn::SUCCESS;
}

CallbackReturn PlaneDetector::on_error(const rclcpp_lifecycle::State & state)
{
    RCLCPP_FATAL(get_logger(), "Error Processing from %s", state.label().c_str());
    return CallbackReturn::SUCCESS;
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    if (rclcpp::ok())
    {
        rclcpp::executors::SingleThreadedExecutor executor;
        rclcpp::NodeOptions options;
        auto node = std::make_shared<PlaneDetector>(options);
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