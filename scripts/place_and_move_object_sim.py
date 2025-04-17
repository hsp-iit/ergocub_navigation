import rclpy
from rclpy.node import Node

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Twist
from gazebo_msgs.srv import SpawnEntity
from gazebo_msgs.msg import ModelState

import tf_transformations
import math


class RvizGazeboSpawner(Node):
    def __init__(self):
        super().__init__('rviz_gazebo_spawner')

        # Interactive Marker Server
        self.server = InteractiveMarkerServer(self, "object_marker")

        # Gazebo interfaces
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.state_publisher = self.create_publisher(ModelState, '/gazebo/set_model_state', 10)

        self.marker_count = 0
        self.create_interactive_marker()

    def create_interactive_marker(self):
        marker = InteractiveMarker()
        marker.header.frame_id = "map"
        marker.name = "spawn_marker"
        marker.description = "Drag to place. Rotate to set velocity dir."
        marker.scale = 1.0

        # Initial pose
        marker.pose.position.x = 1.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.5

        # Show a cube to represent the object
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.scale.x = 0.3
        box_marker.scale.y = 0.3
        box_marker.scale.z = 0.3
        box_marker.color.r = 0.0
        box_marker.color.g = 1.0
        box_marker.color.b = 0.0
        box_marker.color.a = 0.8

        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append(box_marker)
        marker.controls.append(box_control)

        # Allow move in XY plane
        move_control = InteractiveMarkerControl()
        move_control.name = "move_xy"
        move_control.orientation.w = 1.0
        move_control.orientation.x = 0.0
        move_control.orientation.y = 1.0
        move_control.orientation.z = 0.0
        move_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        marker.controls.append(move_control)

        # Allow Z translation
        move_z = InteractiveMarkerControl()
        move_z.name = "move_z"
        move_z.orientation.w = 1.0
        move_z.orientation.x = 0.0
        move_z.orientation.y = 0.0
        move_z.orientation.z = 1.0
        move_z.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        marker.controls.append(move_z)

        # Allow rotation in Yaw (to set velocity direction)
        rotate_yaw = InteractiveMarkerControl()
        rotate_yaw.name = "rotate_z"
        rotate_yaw.orientation.w = 1.0
        rotate_yaw.orientation.x = 0.0
        rotate_yaw.orientation.y = 1.0
        rotate_yaw.orientation.z = 0.0
        rotate_yaw.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        marker.controls.append(rotate_yaw)

        self.server.insert(marker, feedback_callback=self.handle_feedback)
        self.server.applyChanges()



    def handle_feedback(self, feedback):
        if feedback.event_type == feedback.MOUSE_UP:
            pose = feedback.pose
            self.spawn_box('box_1',pose)
            self.set_velocity('box_1',pose)

    def spawn_box(self, name, pose):
        sdf = f"""
        <sdf version="1.6">
          <model name="{name}">
            <pose>0 0 0 0 0 0</pose>
            <link name="link">
              <pose>0 0 0.5 0 0 0</pose>
              <collision name="collision">
                <geometry>
                  <box><size>0.2 0.2 0.2</size></box>
                </geometry>
              </collision>
              <visual name="visual">
                <geometry>
                  <box><size>0.2 0.2 0.2</size></box>
                </geometry>
              </visual>
            </link>
          </model>
        </sdf>
        """

        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity service...')

        request = SpawnEntity.Request()
        request.name = name
        request.xml = sdf
        request.robot_namespace = name
        request.initial_pose = pose
        request.reference_frame = "map"

        self.spawn_client.call_async(request)

    def set_velocity(self, name, pose):
        # Use yaw from quaternion to calculate direction
        orientation = pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        vx = math.cos(yaw) * 1.0
        vy = math.sin(yaw) * 1.0

        state = ModelState()
        state.model_name = name
        state.twist.linear.x = vx
        state.twist.linear.y = vy
        state.twist.linear.z = 0.0
        self.state_publisher.publish(state)


def main():
    rclpy.init()
    node = RvizGazeboSpawner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
