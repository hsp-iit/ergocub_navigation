#include "FootstepsViewer/FootstepsViewerRos.hpp"

FootstepsViewerRos::FootstepsViewerRos() : rclcpp::Node("footstep_viewer_node")
{   
    m_rightFootprintsMarkersPub = this->create_publisher<visualization_msgs::msg::MarkerArray>(m_rightFootprintsTopicName, 10);
    m_leftFootprintsMarkersPub = this->create_publisher<visualization_msgs::msg::MarkerArray>(m_leftFootprintsTopicName, 10);
};

bool FootstepsViewerRos::publishMarkers(const yarp::os::Bottle& data){
        std::cout << "publishMarkers" << std::endl;
        std::cout << data.size() << std::endl;
        auto leftSteps = data.get(0).asList();
        auto rightSteps = data.get(1).asList();
        if (leftSteps->size() == 0 || rightSteps->size()==0)
        {
            RCLCPP_INFO(this->get_logger(), "One of the Step array is empty");
            return false;
        }

        visualization_msgs::msg::MarkerArray right_marker_array;
        visualization_msgs::msg::MarkerArray left_marker_array;
        //Clear the previous markers
        visualization_msgs::msg::Marker clear_msg;
        clear_msg.id = 0;
        clear_msg.ns = "my_namespace";
        clear_msg.action = visualization_msgs::msg::Marker::DELETEALL;
        
        right_marker_array.markers.push_back(clear_msg);
        left_marker_array.markers.push_back(clear_msg);
        m_leftFootprintsMarkersPub->publish(left_marker_array);
        m_rightFootprintsMarkersPub->publish(right_marker_array);

        left_marker_array.markers.clear();
        right_marker_array.markers.clear();
        builtin_interfaces::msg::Time timestamp = now();
        //LEFT
        std::cout << "Left Loop" << std::endl;
        //RCLCPP_INFO(this->get_logger(), "Left Loop");
        for (size_t i = 0; i < leftSteps->size(); ++i)
        {
            visualization_msgs::msg::Marker tmp_marker_msg;
            tmp_marker_msg.header.frame_id = "odom";
            tmp_marker_msg.id = i;
            tmp_marker_msg.ns = "my_namespace";
            tmp_marker_msg.header.stamp = timestamp;
            tmp_marker_msg.scale.x = 0.05;
            tmp_marker_msg.scale.y = 0.05;
            tmp_marker_msg.scale.z = 0.05;
            // Color for left foot
            tmp_marker_msg.color.r = 0.0;
            tmp_marker_msg.color.g = 1.0;
            tmp_marker_msg.color.b = 0.0;
            tmp_marker_msg.color.a = 1.0;
            tmp_marker_msg.type = visualization_msgs::msg::Marker::ARROW;
            tmp_marker_msg.pose.position.x = leftSteps->get(i).asList()->get(0).asFloat64();
            tmp_marker_msg.pose.position.y = leftSteps->get(i).asList()->get(1).asFloat64();
            tmp_marker_msg.pose.position.z = 0.0;
            tf2::Quaternion q;
            q.setRPY(0, 0, leftSteps->get(i).asList()->get(2).asFloat64());
            tmp_marker_msg.pose.orientation = tf2::toMsg(q);
            tmp_marker_msg.frame_locked = true;
            tmp_marker_msg.action = visualization_msgs::msg::Marker::ADD;
            //Populate the marker with atleast one mesh point
            geometry_msgs::msg::Point cube_center;
            cube_center.x = 0.0;
            cube_center.y = 0.0;
            cube_center.z = 0.0;
            tmp_marker_msg.points.push_back(cube_center);
            cube_center.x = 0.1;
            cube_center.y = 0.0;
            cube_center.z = 0.0;
            tmp_marker_msg.points.push_back(cube_center);
            //save marker in the array
            left_marker_array.markers.push_back(tmp_marker_msg);
        }
        //RCLCPP_INFO(this->get_logger(), "Publishing Left");
        std::cout << "Left Publish" << std::endl;
        m_leftFootprintsMarkersPub->publish(left_marker_array);

        //RIGHT
        std::cout << "Right Loop" << std::endl;
        //RCLCPP_INFO(this->get_logger(), "Right Loop");

        //tmp_marker_msg.points.clear();
        for (size_t i = 0; i < rightSteps->size(); ++i)
        {
            visualization_msgs::msg::Marker tmp_marker_msg;
            tmp_marker_msg.header.frame_id = "odom";
            tmp_marker_msg.id = i;
            tmp_marker_msg.ns = "my_namespace";
            tmp_marker_msg.header.stamp = timestamp;
            tmp_marker_msg.scale.x = 0.05;
            tmp_marker_msg.scale.y = 0.05;
            tmp_marker_msg.scale.z = 0.05;
            // Color for left foot
            tmp_marker_msg.color.r = 1.0;
            tmp_marker_msg.color.g = 0.0;
            tmp_marker_msg.color.b = 0.0;
            tmp_marker_msg.color.a = 1.0;
            tmp_marker_msg.type = visualization_msgs::msg::Marker::ARROW;
            tmp_marker_msg.pose.position.x = rightSteps->get(i).asList()->get(0).asFloat64();
            tmp_marker_msg.pose.position.y = rightSteps->get(i).asList()->get(1).asFloat64();
            tmp_marker_msg.pose.position.z = 0.0;
            tf2::Quaternion q;
            q.setRPY(0, 0, rightSteps->get(i).asList()->get(2).asFloat64());
            tmp_marker_msg.pose.orientation = tf2::toMsg(q);
            tmp_marker_msg.frame_locked = true;
            tmp_marker_msg.action = visualization_msgs::msg::Marker::ADD;
            //Populate the marker with atleast 2 mesh point for ARROW
            geometry_msgs::msg::Point cube_center;
            cube_center.x = 0.0;
            cube_center.y = 0.0;
            cube_center.z = 0.0;
            tmp_marker_msg.points.push_back(cube_center);
            cube_center.x = 0.1;
            cube_center.y = 0.0;
            cube_center.z = 0.0;
            tmp_marker_msg.points.push_back(cube_center);
            //save marker in the array
            right_marker_array.markers.push_back(tmp_marker_msg);
        }

        //RCLCPP_INFO(this->get_logger(), "Publishing Right");
        std::cout << "Right Publish" << std::endl;
        //publish
        m_rightFootprintsMarkersPub->publish(right_marker_array);
        std::cout << "Exiting" << std::endl;
};
