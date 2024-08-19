#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rms_msgs.srv import PCDCollection

from os import path
import sys
import tf2_ros

sys.path.append(path.abspath(path.join(path.dirname(__file__), "../../../../")))

from modules.collection import *
from modules.manipulators import *


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

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
   

    def _execute_callback(self, request, response):
        """
        
        """
        manipulator = request.manipulator
        read_path = request.read_path


        self._pcd_subscription = self.create_subscription(
            PointCloud2,
            f"/{manipulator}/camera/depth/color/points",
            self._pcd_callback,
            10
        )

        viewpoints = get_viewpoints(read_path)
        bot = get_vx_bot(manipulator)

        #
        for view in viewpoints:
            move_vx_bot_to_viewpoint(bot, x=view[1], y=view[2], z=view[3])

        move_vx_bot_to_home(bot)
        move_vx_bot_to_sleep(bot)
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
