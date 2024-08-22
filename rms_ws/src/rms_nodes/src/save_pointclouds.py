#!/usr/bin/env python3

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rms_msgs.msg import PointCloudStamped

from os import path
import sys

sys.path.append(path.abspath(path.join(path.dirname(__file__), "../../../../")))

from modules.collection import (
    check_path,
    create_path,
    save_pointcloud,
)


class SavePointClouds(Node):
    """
    
    """
    def __init__(self):
        """
        
        """
        super().__init__("save_pointclouds")

        self.declare_parameter("pointcloud_publisher_topic", "/rms/pointcloud/save")
        self._pointcloud_publisher_topic = self.get_parameter("pointcloud_publisher_topic").get_parameter_value().string_value

        self._pointcloud_subscription = self.create_subscription(
            PointCloudStamped,
            self._pointcloud_publisher_topic,
            self._pointcloud_callback,
            10
        )

        self.get_logger().info("ready to save pointclouds")

    
    def _pointcloud_callback(self, msg: PointCloudStamped):
        """
        Save the PointCloud2 in the PointCloudStamped message at the file_path as file_name.
        
        Args:
            msg (PointCloudStamped): The ROS2 PointCloudStamped message that is to be saved.
        """
        self.get_logger().info(f"received {msg.file_name}")
        file_path = path.abspath(path.join(path.expanduser("~/"), msg.file_path))
        if not check_path(file_path):
            create_path(file_path) 

        file_path_ = path.abspath(path.join(file_path, msg.file_name))
        save_pointcloud(msg.pointcloud, file_path_)
        self.get_logger().info(f"saved {msg.file_name} at {file_path}")


def main(args=None):
    """
    
    """
    try:
        rclpy.init(args=args)
        node = SavePointClouds()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
