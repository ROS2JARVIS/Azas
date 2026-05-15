#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateRelay(Node):
    def __init__(self):
        super().__init__("joint_state_relay")
        self.declare_parameter("input_topic", "/dsr01/joint_states")
        self.declare_parameter("output_topic", "/joint_states")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        self.publisher = self.create_publisher(JointState, output_topic, 10)
        self.subscription = self.create_subscription(
            JointState,
            input_topic,
            self.callback,
            10,
        )
        self.get_logger().info(f"Relaying {input_topic} -> {output_topic}")

    def callback(self, msg):
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
