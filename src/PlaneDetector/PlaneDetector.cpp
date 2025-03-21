#include "PlaneDetector/PlaneDetector.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <tf2_eigen/tf2_eigen.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using std::placeholders::_1;
using namespace std::chrono_literals;

void PlaneDetector::pc_callback(const sensor_msgs::msg::PointCloud2::ConstPtr& pc_in)
{
    // TODO: Check if the robot is in double support
    //if (!(m_left_foot_contact && m_right_foot_contact))
    //{
    //    return;
    //}
    // Transform
    sensor_msgs::msg::PointCloud2 realsense_cloud;
    std::string transform_error;
    geometry_msgs::msg::TransformStamped realsense_to_foot_tf, realsense_frame_to_XYZ;
    if (m_tf_buffer_in->canTransform(
                pc_in->header.frame_id,
                m_realsense_frame,
                pc_in->header.stamp,
                tf2::durationFromSec(0.02),  
                & transform_error ))
    {
        try
        {
            realsense_cloud = m_tf_buffer_in->transform(*pc_in, m_realsense_frame, tf2::durationFromSec(0.0));
            realsense_to_foot_tf = m_tf_buffer_in->lookupTransform(m_contact_frame, m_realsense_frame, rclcpp::Time(0));
            realsense_frame_to_XYZ = m_tf_buffer_in->lookupTransform("realsense", "realsense_rgb_frame", rclcpp::Time(0));
        }
        catch(const std::exception& e)
        {
           RCLCPP_WARN(this->get_logger(), "Transform exception: %s \n", e.what());
           return;
        }
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Cannot transform %s to %s: %s \n", pc_in->header.frame_id, m_realsense_frame, transform_error);
        return;
    }

    // do Ransac only for the first N samples
    if (m_tf_vec.size() >= m_sample_size)
    {
        sensor_msgs::msg::PointCloud2 compensated_cloud = realsense_cloud;
        compensated_cloud.header.frame_id = m_frame_name;
        m_pointcloud_pub->publish(compensated_cloud);
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(realsense_cloud, *in_cloud);    // Cloud expressed in camera frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    /*****************Cut roof (in camera frame)*/
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(in_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(m_filter_z_low, m_filter_z_high);
    pass.filter(*filtered_cloud);

    /*****************Transform in the robot foot frame*/
    sensor_msgs::msg::PointCloud2 ground_cloud;
    pcl::toROSMsg(*filtered_cloud, ground_cloud);
    try
    {
        tf2::doTransform(ground_cloud, ground_cloud, realsense_to_foot_tf);
    }
    catch(const std::exception& e)
    {
       RCLCPP_WARN(this->get_logger(), "Cannot transform: %s \n", e.what());
       return;
    }
    pcl::fromROSMsg(ground_cloud, *filtered_cloud);

    /*****************Detect ground plane RANSAC*/
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.05);
    
    seg.setInputCloud(filtered_cloud);
    seg.segment(*inliers, *coefficients);
    
    if (inliers->indices.size() == 0)
    {
        RCLCPP_ERROR(get_logger(), "Could not estimate a planar model for the given cloud.");
        return;
    }
    // Plane coefficients: 
    //RCLCPP_INFO_STREAM(get_logger(), "Model coefficients: " << coefficients->values[0] << " " 
    //                                    << coefficients->values[1] << " "
    //                                    << coefficients->values[2] << " " 
    //                                    << coefficients->values[3]);

    double A = coefficients->values[0];
    double B = coefficients->values[1];
    double C = coefficients->values[2];
    RCLCPP_INFO_STREAM(get_logger(), "plane coefficients: A " << A << " B " 
                                        << B << " C "
                                        << C );
    
    RCLCPP_INFO_STREAM(get_logger(), "Model inliers: " << inliers->indices.size());
    // Extract plane points
    if (m_debug_publish)
    {
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (filtered_cloud);
        extract.setIndices (inliers);
        extract.setNegative (false);
        pcl::PointCloud<pcl::PointXYZ>::Ptr plane_points (new pcl::PointCloud<pcl::PointXYZ>);
        extract.filter(*plane_points);

        pcl::toROSMsg(*plane_points, ground_cloud);
        ground_cloud.header.stamp = pc_in->header.stamp;
        m_plane_pub->publish(ground_cloud);
    }
    
    /*****************Get RPY angles from plane coeff*/
    double tmp = std::sqrt(A*A + B*B + C*C);    // This should be = 1
    double pitch = std::acos(A / tmp) - M_PI_2; // We remove M_PI_2 since Ransac adds a strange rotation
    double roll = - (std::acos(B / tmp) - M_PI_2);
    double yaw = std::acos(C / tmp);
    RCLCPP_INFO(get_logger(), "roll: %f pitch: %f yaw: %f ", roll, pitch, yaw);
    // New computation
    //double pitch_ = std::atan2(C, std::sqrt(A * A + B * B));
    //double roll_ = std::atan2(B, A);
    //double yaw_ = std::atan2(A, C);
    //RCLCPP_INFO(get_logger(), "NEW FORMULATION roll: %f pitch: %f yaw: %f ", roll_, pitch_, yaw_);
    
    /*****************Publish static frame corrected by RPY angles*/
    geometry_msgs::msg::TransformStamped msg;
    msg.header.frame_id = m_realsense_frame;
    //msg.header.frame_id = "";
    msg.child_frame_id = m_frame_name;
    msg.header.stamp = ground_cloud.header.stamp;
    msg.transform.translation.x = 0.0;
    msg.transform.translation.y = 0.0;
    msg.transform.translation.z = 0.0;
    tf2::Quaternion quat;
    quat.setRPY(roll, pitch, yaw);
    msg.transform.rotation = tf2::toMsg(quat);
    m_tf_vec.push_back({roll, pitch});
    RCLCPP_INFO(get_logger(), "tf msg rotation x: %f y: %f z: %f w: %f", msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w);

    if (m_tf_vec.size() >= m_sample_size)
    {
        m_avg_tf = average_pitch(m_tf_vec, ground_cloud.header.stamp);
        m_static_tf_broadcaster->sendTransform(m_avg_tf);
        auto m_avg_tf_ = tf2::eigenToTransform(tf2::transformToEigen(realsense_frame_to_XYZ.transform) * tf2::transformToEigen(m_avg_tf.transform));
        m_avg_tf_.header.stamp = m_avg_tf.header.stamp;
        m_avg_tf_.child_frame_id = "realsense_compensated";
        m_avg_tf_.header.frame_id = m_avg_tf.child_frame_id;
        m_static_tf_broadcaster->sendTransform(m_avg_tf_);
    }
    else
    {
        m_tf_broadcaster->sendTransform(msg);
    }

    try
    {
        //sensor_msgs::msg::PointCloud2 compensated_cloud = m_tf_buffer_in->transform(
        //                                            realsense_cloud, "compensated_realsense_frame", tf2::durationFromSec(0.05));
        sensor_msgs::msg::PointCloud2 compensated_cloud = realsense_cloud;
        compensated_cloud.header.frame_id = m_frame_name;
        m_pointcloud_pub->publish(compensated_cloud);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}

geometry_msgs::msg::TransformStamped PlaneDetector::average_pitch(std::vector<std::tuple<double, double>> tf_vec_in, builtin_interfaces::msg::Time stmp)
{
    geometry_msgs::msg::TransformStamped avg_tf;
    double roll_sum, pitch_sum;
    for (size_t i = 0; i < tf_vec_in.size(); i++)
    {
        roll_sum += std::get<0>(tf_vec_in[i]);
        pitch_sum += std::get<1>(tf_vec_in[i]);
    }
    avg_tf.header.frame_id = m_realsense_frame;
    avg_tf.header.stamp = stmp;
    avg_tf.child_frame_id = m_frame_name;
    tf2::Quaternion q;
    double avg_roll = roll_sum/tf_vec_in.size();
    double avg_pitch = pitch_sum/tf_vec_in.size();
    RCLCPP_INFO_STREAM(this->get_logger(), "Average pitch: " << avg_pitch << " roll: " << avg_roll);
    q.setRPY(avg_roll, avg_pitch, 0.0);
    avg_tf.transform.rotation = tf2::toMsg(q);
    // could be skipped, but do it to be sure we have a 0 traslation
    avg_tf.transform.translation.x = 0.0;
    avg_tf.transform.translation.y = 0.0;
    avg_tf.transform.translation.z = 0.0;
    return avg_tf;
}

std::vector<geometry_msgs::msg::TransformStamped> PlaneDetector::median_filter(std::vector<geometry_msgs::msg::TransformStamped> tf_vec_in)
{
    int filter_size = 3;
    for (size_t i = 0; i < tf_vec_in.size(); i++)
    {
        /* code */
    }
    
    
}

PlaneDetector::PlaneDetector(const rclcpp::NodeOptions & options) : rclcpp_lifecycle::LifecycleNode("floor_detector_node", options)
{
    declare_parameter("head_frame", "realsense");
    declare_parameter("contact_frame", "r_sole");
    declare_parameter("pointcloud_topic", "/camera/depth/color/points");
    declare_parameter("pub_topic", "/adjusted_depth_pc");
    declare_parameter("filter_z_low", -2.0);
    declare_parameter("filter_z_high", -0.7);

    
    m_tf_buffer_in = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer_in);
    m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    m_static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
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
    
    //Publisher
    m_pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(m_pub_topic, 10);
    m_plane_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/plane_detector/ground_plane", 10);

    return CallbackReturn::SUCCESS;
}

/*void PlaneDetector::right_foot_callback(const geometry_msgs::msg::WrenchStamped::ConstPtr &msg)
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
*/

CallbackReturn PlaneDetector::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Activating");
    m_pointcloud_pub->on_activate();
    m_plane_pub->on_activate();
    return CallbackReturn::SUCCESS;
}

CallbackReturn PlaneDetector::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Deactivating");
    m_pointcloud_pub->on_deactivate();
    m_plane_pub->on_deactivate();
    return CallbackReturn::SUCCESS;
}

CallbackReturn PlaneDetector::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Cleaning Up");
    m_pointcloud_pub.reset();
    m_plane_pub.reset();
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