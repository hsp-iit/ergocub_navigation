#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState


class JointStateRepublisher(Node):
    def __init__(self):
        super().__init__(
            'joint_state_republisher',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
            automatically_declare_parameters_from_overrides=True,
        )

        self.sub = self.create_subscription(
            JointState,
            '/joint_states_raw',
            self.on_joint_states,
            10,
        )

        self.pub = self.create_publisher(
            JointState,
            '/joint_states',
            10,
        )

        self.warned_waiting_for_clock = False

        self.get_logger().info(
            'JointState republisher started: /joint_states_raw -> /joint_states'
        )

    def on_joint_states(self, msg: JointState):
        now = self.get_clock().now()

        # With use_sim_time=True, ROS time is zero until /clock is received.
        # Do not publish zero-stamped joint states.
        if now.nanoseconds == 0:
            if not self.warned_waiting_for_clock:
                self.get_logger().warn(
                    'ROS time is still zero. Waiting for /clock before publishing /joint_states...'
                )
                self.warned_waiting_for_clock = True
            return

        out = JointState()
        out.header = msg.header
        out.header.stamp = now.to_msg()

        out.name = msg.name
        out.position = msg.position
        out.velocity = msg.velocity
        out.effort = msg.effort

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateRepublisher()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
