#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rms_msgs.action import PCDCollection

from os import path
import sys

sys.path.append(path.abspath(path.join(path.dirname(__file__), "../../../../")))

from modules.collection import get_viewpoints # type: ignore


class PCDCollectionServer(Node):
    """
    
    """
    def __init__(self):
        """
        
        """
        super().__init__("pcd_collection_server")
        self.action = ActionServer(
            self,
            PCDCollection,
            "pdc_collection",
            self.execute_callback
        )
    

    def execute_callback(self, goal_handle):
        """
        
        """
        self.get_logger().info("executing goal")

        feedback_handle = PCDCollection.Feedback()
        feedback_handle.progress = 0.0
        manipulator = goal_handle.request.manipulator
        read_path = goal_handle.request.read_path
        write_directory = goal_handle.request.write_directory

        # get viewpoints
        viewpoints = get_viewpoints(read_path)

        #
        goal_handle.succeed()
        result_handle = PCDCollection.Result()
        return result_handle


def main(args=None):
    """
    
    """
    try:
        rclpy.init(args=args)
        node = PCDCollectionServer()
        rclpy.spin(node)
    except:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
