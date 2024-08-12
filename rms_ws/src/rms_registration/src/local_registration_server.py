#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class LocalRegistrationServer(Node):
    """
    
    """

    def __init__(self):
        """
        
        
        """

        super().__init__("local_registration")
        self.srv = self.create_service(

        )


def main(args=None):    
    try:
        rclpy.init(args=args)
        node = LocalRegistrationServer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("keyboard interrupt, shutting down ...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
