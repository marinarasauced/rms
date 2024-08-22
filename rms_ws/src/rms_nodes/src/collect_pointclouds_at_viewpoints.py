#!/usr/vin/env python3

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Point
from rms_msgs.action import PointCloudCollection
from rms_msgs.msg import PointCloudStamped
from sensor_msgs.msg import PointCloud2

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
        self._pointcloud_publisher_topic = self.get_parameter("_pointcloud_publisher_topic").get_parameter_value().string_value
        self._pointcloud_subscription_topic = self.get_parameter("pointcloud_topic").get_parameter_value().string_value
        self._collection_action = self.get_parameter("collection_action").get_parameter_value().string_value
        
        self.offset = Point()
        self.offset.x = self.get_parameter("offset_x").get_parameter_value().double_value
        self.offset.y = self.get_parameter("offset_y").get_parameter_value().double_value
        self.offset.z = self.get_parameter("offset_z").get_parameter_value().double_value
        self.wait_duration = self.get_parameter("wait_duration").get_parameter_value().double_value

        self.bot = get_vx_bot(self.robot_model)

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
            self._pointcloud2_topic,
            10,
            self._pointcloud_callback
        )

        self._collection_server = ActionServer(
            self,
            PointCloudCollection,
            self._collection_action,
            self._collection_execute
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

    
    def _pointcloud_callback(self, msg: PointCloud2):
        """
        Publish the PointCloud2 
        
        Args:
            msg (PointCloud2): The ROS2 PointCloudStamped message that is to be republished.
        """
        self.pointcloud = msg
        if self.file_path and self.pointcloud and self.status:
            self._pointcloud_publish()
            self.status = 0

    
    def _collection_execute(self, goal_handle):
        """
        
        """
        goal = goal_handle.request
        feedback = PointCloudCollection.Feedback()
        result = PointCloudCollection.Result()

        if not goal.viewpoints or not goal.scans_path:
            self.get_logger().error("invalid goal: viewpoints or scans_path is empty")
            result.success = 0.0
            goal_handle.abort(result)
            return
        self.file_path = goal.scans_path
        
        self.get_logger().info("collecting pointclouds at viewpoints")
        feedback.progress = 0.0
        goal_handle.publish_feedback(feedback)
        for viewpoint in goal.viewpoints:
            move_vx_bot_to_viewpoint(self.bot, viewpoint, self.offset, self.wait_duration)

            self.pointcloud = None
            self.status = 1
            tic = time.time()
            while self.pointcloud is None:
                toc = time.time()
                rclpy.spin_once(self, timeout_sec=0.1)
                if toc - tic > 5.0:
                    self.get_logger().info("timed out waiting for pointcloud")
                    break
            
            self.counter += 1
            feedback.progress = (self.counter - 1) / len(goal.viewpoints)
            goal_handle.publish_feedback(feedback)
        
        move_vx_bot_to_home(self.bot)
        move_vx_bot_to_sleep(self.bot)

        goal_handle.succeed()
        result.success = 1
        return result


    def destroy_node(self):
        """
        
        """
        self.bot.shutdown()
        super().destroy_node()


def main(args=None):
    """
    
    """
    try:
        rclpy.init(args=args)
        node = PointCloudCollectionServer()
        rclpy.spin(node)
    except:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
