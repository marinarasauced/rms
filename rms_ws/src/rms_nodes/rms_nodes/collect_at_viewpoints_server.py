#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from sensor_msgs.msg import PointCloud2
from rms_msgs.action import PCDCollection

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
        self._collection_server = ActionServer(
            self,
            PCDCollection,
            "viewpoint_collection",
            self._execute_callback
        )

        self.declare_parameter("robot_model", "vx000")
        self.manipulator = self.get_parameter("robot_model").get_parameter_value().string_value
        self._pcd_topic = "camera/camera/depth/color/points"

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.counter = 1
        self.data = None
        self.status = False

    def _execute_callback(self, goal_handle):
        """
        
        """
        feedback_handle = PCDCollection.Feedback()
        result_handle = PCDCollection.Result()

        self.read_path = goal_handle.request.read_path
        self.write_path = goal_handle.request.write_path
        viewpoints = get_viewpoints(self.read_path)
        bot = get_vx_bot(self.manipulator)

        self._pcd_subscription = self.create_subscription(
            PointCloud2,
            self._pcd_topic,
            self._pcd_callback,
            10
        )
        
        feedback_handle.progress = 0.0
        goal_handle.publish_feedback(feedback_handle)
        for view in viewpoints:
            move_vx_bot_to_viewpoint(bot, x=view[1], y=view[2], z=view[3])
            self._wait_for_pointcloud()

            self.counter += 1
            feedback_handle.progress = (self.counter - 1) / len(viewpoints)
            goal_handle.publish_feedback(feedback_handle)

        move_vx_bot_to_home(bot)
        move_vx_bot_to_sleep(bot)
        #

        goal_handle.succeed()
        result_handle.success = True
        return result_handle
    

    def _pcd_callback(self, msg: PointCloud2):
        """
        
        """
        self.data = msg
        get_pointcloud(self.data, self.status, self.manipulator, self.counter, self.write_path)

    
    def _wait_for_pointcloud(self):
        """
        
        """
        self.data = None
        self.status = True
        tic = time.time()
        while self.data is None:
            toc = time.time()
            rclpy.spin_once(self, timeout_sec=0.1)
            if toc - tic > 5.0:
                break


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
