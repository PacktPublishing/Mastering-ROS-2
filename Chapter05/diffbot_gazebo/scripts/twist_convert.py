#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist

class TwistToTwistStampedNode(Node):
    def __init__(self):
        super().__init__('twist_to_twist_stamped_node')

        # Declare and read parameters for topic names
        self.declare_parameter('input_topic', '/twist')
        self.declare_parameter('output_topic', '/twist_stamped')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        # Subscriber to Twist topic
        self.subscription = self.create_subscription(
            Twist,
            input_topic,
            self.twist_callback,
            10
        )

        # Publisher for TwistStamped topic
        self.publisher = self.create_publisher(TwistStamped, output_topic, 10)

        self.get_logger().info(f"Subscribed to: {input_topic}")
        self.get_logger().info(f"Publishing to: {output_topic}")

    def twist_callback(self, msg: Twist):
        # Create a TwistStamped message from Twist
        twist_stamped_msg = TwistStamped()
        twist_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        twist_stamped_msg.header.frame_id = 'base_link'  # Set frame_id as needed
        twist_stamped_msg.twist.linear = msg.linear
        twist_stamped_msg.twist.angular = msg.angular

        # Publish the TwistStamped message
        self.publisher.publish(twist_stamped_msg)
        self.get_logger().debug("Published TwistStamped message")

def main(args=None):
    rclpy.init(args=args)

    node = TwistToTwistStampedNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly')
    except Exception as e:
        node.get_logger().error(f"Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

