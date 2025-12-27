#!/usr/bin/env python3

"""
Simple ROS 2 Service Example

This example demonstrates how to create a basic service server and client in Python.
The service adds two integers together.
"""

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class SimpleServiceServer(Node):
    """
    A simple service server that provides an 'add_two_ints' service.
    """

    def __init__(self):
        # Initialize the node with the name 'simple_service_server'
        super().__init__('simple_service_server')

        # Create a service that uses the AddTwoInts interface
        # The 'add_two_ints_callback' method will be called when a request is received
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback)

        # Log a message when the service starts
        self.get_logger().info('Simple service server started')

    def add_two_ints_callback(self, request, response):
        """
        Callback function that is called when a service request is received.
        Performs the addition and returns the result.
        """
        # Perform the addition
        response.sum = request.a + request.b

        # Log the request and response
        self.get_logger().info(f'Request received: {request.a} + {request.b}')
        self.get_logger().info(f'Sending response: {response.sum}')

        # Return the response
        return response


def main(args=None):
    """
    Main function that initializes the ROS 2 client library, creates the service node,
    spins it to keep it running, and handles cleanup when done.
    """
    # Initialize the ROS 2 client library
    rclpy.init(args=args)

    # Create the service server node
    simple_service_server = SimpleServiceServer()

    try:
        # Keep the node running until interrupted
        rclpy.spin(simple_service_server)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        pass
    finally:
        # Clean up the node and shutdown the client library
        simple_service_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()