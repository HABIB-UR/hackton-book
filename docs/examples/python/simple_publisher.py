#!/usr/bin/env python3

"""
Simple ROS 2 Publisher Example

This example demonstrates how to create a basic publisher node in Python.
The node publishes a simple message to a topic at a regular interval.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimplePublisher(Node):
    """
    A simple publisher node that publishes messages to a topic.
    """

    def __init__(self):
        # Initialize the node with the name 'simple_publisher'
        super().__init__('simple_publisher')

        # Create a publisher for String messages on the 'chatter' topic
        self.publisher = self.create_publisher(String, 'chatter', 10)

        # Create a timer that calls the timer_callback method every 0.5 seconds
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter to keep track of the number of messages published
        self.i = 0

        # Log a message when the node starts
        self.get_logger().info('SimplePublisher node started')

    def timer_callback(self):
        """
        Callback function that is called by the timer at regular intervals.
        Creates and publishes a message to the 'chatter' topic.
        """
        # Create a String message
        msg = String()
        msg.data = f'Hello ROS 2 World: {self.i}'

        # Publish the message
        self.publisher.publish(msg)

        # Log the published message
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment the counter
        self.i += 1


def main(args=None):
    """
    Main function that initializes the ROS 2 client library, creates the node,
    spins it to keep it running, and handles cleanup when done.
    """
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create the publisher node
    simple_publisher = SimplePublisher()

    try:
        # Keep the node running until interrupted
        rclpy.spin(simple_publisher)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        pass
    finally:
        # Clean up the node and shutdown the client library
        simple_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()