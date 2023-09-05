#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_2d_msgs/msg/pose2_d_stamped.hpp"
#include "nav_2d_msgs/msg/path2_d.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "yarp/os/Bottle.h"

#include <chrono>

class FootstepsViewerRos : public rclcpp::Node
{
private:
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_rightFootprintsMarkersPub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_leftFootprintsMarkersPub;
    const std::string m_rightFootprintsTopicName = "/footstep_viewer_node/right_footprints";
    const std::string m_leftFootprintsTopicName = "/footstep_viewer_node/left_footprints";
    
public:
    FootstepsViewerRos();

    bool publishMarkers(const yarp::os::Bottle& data);
};  //End of class FootstepsViewerRos