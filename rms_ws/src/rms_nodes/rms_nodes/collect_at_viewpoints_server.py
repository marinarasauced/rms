#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rms_msgs.srv import PCDCollection

from os import path
import sys

sys.path.append(path.abspath(path.join(path.dirname(__file__), "../../../../")))

from modules.collection import *


class ViewpointCollectionServer(Node):
    """
    
    """
    def __init__(self):
        """
        
        """
        super().__init__("viewpoint_collection_server")
        self._collection_server = self.create_service(
            PCDCollection,
            "viewpoint_collection",
            self._execute_callback
        )
   

    def _execute_callback(self, request, response):
        """
        
        """
        manipulator = request.manipulator


        # self._pcd_subscription = self.create_subscription(
        #     PointCloud2,
        #     f"/{manipulator}/camera/depth/color/points",
        #     self._pcd_callback,
        #     10
        # )

        # feedback_handle = PCDCollection.Feedback()
        # feedback_handle.progress = 0.0
        # manipulator = goal_handle.request.manipulator
        # read_path = goal_handle.request.read_path
        # write_directory = goal_handle.request.write_directory

        # # get viewpoints
        # viewpoints = get_viewpoints(read_path)

        #
        return response
    

    def _pcd_callback(self, msg):
        pass


def main(args=None):    
    try:
        rclpy.init(args=args)
        node = ViewpointCollectionServer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("keyboard interrupt, shutting down ...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
