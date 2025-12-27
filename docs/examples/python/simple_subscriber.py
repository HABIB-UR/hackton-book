#!/usr/bin/env python3

"""
Simple ROS 2 Subscriber Example

This example demonstrates how to create a basic subscriber node in Python.
The node subscribes to messages from a topic and logs them.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleSubscriber(Node):
    """
    A simple subscriber node that listens to messages from a topic.
    """

    def __init__(self):
        # Initialize the node with the name 'simple_subscriber'
        super().__init__('simple_subscriber')

        # Create a subscription to String messages on the 'chatter' topic
        # The listener_callback function will be called when a message is received
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)  # QoS profile depth (number of messages to buffer)

        # Prevent unused variable warning
        self.subscription

        # Log a message when the node starts
        self.get_logger().info('SimpleSubscriber node started')

    def listener_callback(self, msg):
        """
        Callback function that is called when a message is received on the topic.
        Logs the received message.
        """
        # Log the received message
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    """
    Main function that initializes the ROS 2 client library, creates the node,
    spins it to keep it running, and handles cleanup when done.
    """
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create the subscriber node
    simple_subscriber = SimpleSubscriber()

    try:
        # Keep the node running until interrupted
        rclpy.spin(simple_subscriber)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        pass
    finally:
        # Clean up the node and shutdown the client library
        simple_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()