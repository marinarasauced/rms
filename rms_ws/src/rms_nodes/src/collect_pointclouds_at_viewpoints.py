#!/usr/vin/env python3

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Point
from rms_msgs.action import PointCloudCollection
from rms_msgs.msg import PointCloudStamped
from sensor_msgs.msg import PointCloud2

import asyncio
import numpy as np
from os import path
import sys
import time

sys.path.append(path.abspath(path.join(path.dirname(__file__), "../../../../")))

from modules.manipulator import (
    get_vx_bot,
    move_vx_bot_to_viewpoint,
    move_vx_bot_to_home,
    move_vx_bot_to_sleep,
)


class PointCloudCollectionServer(Node):
    """
    
    """
    def __init__(self):
        """
        
        """
        super().__init__("pointcloud_collection")

        self.declare_parameter("robot_model", "vx___")
        self.declare_parameter("pointcloud_publisher_topic", "/rms/pointcloud/save")
        self.declare_parameter("pointcloud_subscription_topic", "camera/camera/depth/color/points")
        self.declare_parameter("collection_action", "collect_pointclouds_at_viewpoints")
        self.declare_parameter("offset_x", 0.7)
        self.declare_parameter("offset_y", 0.0)
        self.declare_parameter("offset_z", 0.0)
        self.declare_parameter("wait_duration", 2.0)

        self.robot_model = self.get_parameter("robot_model").get_parameter_value().string_value
        self._pointcloud_publisher_topic = self.get_parameter("pointcloud_publisher_topic").get_parameter_value().string_value
        self._pointcloud_subscription_topic = self.get_parameter("pointcloud_subscription_topic").get_parameter_value().string_value
        self._collection_action = self.get_parameter("collection_action").get_parameter_value().string_value
        
        self.offset = Point()
        self.offset.x = self.get_parameter("offset_x").get_parameter_value().double_value
        self.offset.y = self.get_parameter("offset_y").get_parameter_value().double_value
        self.offset.z = self.get_parameter("offset_z").get_parameter_value().double_value
        self.wait_duration = self.get_parameter("wait_duration").get_parameter_value().double_value

        self.bot = get_vx_bot(self.robot_model)

        self.busy = 0
        self.counter = 1
        self.file_path = None
        self.pointcloud = None
        self.status = 0

        self._pointcloud_publisher = self.create_publisher(
            PointCloudStamped,
            self._pointcloud_publisher_topic,
            10
        )

        self._pointcloud_subscription = self.create_subscription(
            PointCloud2,
            self._pointcloud_subscription_topic,
            self._pointcloud_callback,
            10
        )

        self._collection_server = ActionServer(
            self,
            PointCloudCollection,
            self._collection_action,
            execute_callback=self._collection_execute,
            goal_callback=self._handle_goal,
            cancel_callback=self._handle_cancel,
        )

        self.get_logger().info("ready to collect pointclouds")

    
    def _pointcloud_publish(self):
        """
        Republish a PointCloud2 message as a PointCloudStamped message to save a pointcloud on the registration server machine.
        """
        msg = PointCloudStamped()
        msg.pointcloud = self.pointcloud
        msg.file_path = self.file_path
        msg.file_name = f"{self.robot_model}_{self.counter:03d}.pcd"
        self._pointcloud_publisher.publish(msg)
        self.get_logger().info(f"published pointcloud {msg.file_name}")

    
    def _pointcloud_callback(self, msg: PointCloud2):
        """
        Publish the PointCloud2 
        
        Args:
            msg (PointCloud2): The ROS2 PointCloudStamped message that is to be republished.
        """
        self.pointcloud = msg
        if self.file_path and self.pointcloud and self.status:
            self._pointcloud_publish()
            self.counter = self.counter + 1
            self.status = 0

    
    def _handle_goal(self, goal_handle):
        """
        
        """
        self.get_logger().info("received goal request")
        if self.busy != 0:
            self.get_logger().warning("a goal is already being processed, rejecting new goal")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _handle_cancel(self, goal_handle):
        """
        
        """
        self.get_logger().info("received cancel request")
        return CancelResponse.ACCEPT

    
    def _collection_execute(self, goal_handle):
        """
        Execute the point cloud collection goal.
        """
        self.busy = 1
        self.counter = 1
        self.file_path = None
        self.pointcloud = None
        self.status = 0

        goal = goal_handle.request
        feedback = PointCloudCollection.Feedback()
        result = PointCloudCollection.Result()

        if not goal.viewpoints or not goal.scans_path:
            self.get_logger().error("invalid goal, viewpoints or scans_path is empty")
            result.success = False
            goal_handle.abort()
            self.busy = 0
            return result

        self.get_logger().info("goal accepted")
        self.file_path = goal.scans_path
        feedback.progress = 0.0
        goal_handle.publish_feedback(feedback)

        try:
            for viewpoint in goal.viewpoints:
                move_vx_bot_to_viewpoint(self.bot, viewpoint, self.offset, self.wait_duration)

                self.pointcloud = None
                self.status = 1
                tic = time.time()

                while self.pointcloud is None:
                    toc = time.time()
                    time.sleep(0.1)
                    if toc - tic > self.wait_duration:
                        self.get_logger().info("timed out waiting for pointcloud")
                        break

                if self.pointcloud is not None:
                    feedback.progress = self.counter / len(goal.viewpoints)
                    goal_handle.publish_feedback(feedback)

            goal_handle.succeed()
            result.success = True

        except Exception as e:
            self.get_logger().error(f"error during goal execution: {e}")

            result.success = False
            goal_handle.abort()

        finally:
            time.sleep(1)
            move_vx_bot_to_home(self.bot)
            move_vx_bot_to_sleep(self.bot)
            self.busy = 0
            self.pointcloud = None

        self.get_logger().info("goal completed successfully")
        return result


def main(args=None):
    """
    
    """
    try:
        rclpy.init(args=args)
        node = PointCloudCollectionServer()
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

