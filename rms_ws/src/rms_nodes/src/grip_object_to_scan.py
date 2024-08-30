#!/usr/vin/env python3

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Point
from rms_msgs.action import GripObjectToScan

from os import path
import sys
import time

sys.path.append(path.abspath(path.join(path.dirname(__file__), "../../../../")))

from modules.manipulator import (
    get_vx_bot,
    grip_vx_bot_at_viewpoint,
    move_vx_bot_to_viewpoint,
    move_vx_bot_to_home,
    move_vx_bot_to_sleep,
)


class GripObjectToShowServer(Node):
    """
    
    """
    def __init__(self):
        """
        
        """
        super().__init__("object_gripper")

        self.declare_parameter("robot_model", "vx___")
        self.declare_parameter("grip_action", "grip_object_to_scan")
        self.declare_parameter("offset_x", 0.0)
        self.declare_parameter("offset_y", 0.0)
        self.declare_parameter("offset_z", 100.0)
        self.declare_parameter("wait_duration", 10.0)

        self.robot_model = self.get_parameter("robot_model").get_parameter_value().string_value
        self._grip_action = self.get_parameter("grip_action").get_parameter_value().string_value

        self.offset = Point()
        self.offset.x = self.get_parameter("offset_x").get_parameter_value().double_value
        self.offset.y = self.get_parameter("offset_y").get_parameter_value().double_value
        self.offset.z = self.get_parameter("offset_z").get_parameter_value().double_value
        self.wait_duration = self.get_parameter("wait_duration").get_parameter_value().double_value

        self.bot = get_vx_bot(self.robot_model)

        self.busy = 0
        self.status = 0

        self._grip_server = ActionServer(
            self,
            GripObjectToScan,
            self._grip_action,
            execute_callback=self._collection_execute,
            goal_callback=self._handle_goal,
            cancel_callback=self._handle_cancel,
        )

        self.get_logger().info("ready to grip object")

    
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
        Execute the grip goal.
        """
        self.busy = 1
        self.status = 0

        goal = goal_handle.request
        feedback = GripObjectToScan.Feedback()
        result = GripObjectToScan.Result()

        grip_pose = goal.viewpoints[0]
        show_pose = goal.viewpoints[1]

        if not grip_pose or not show_pose:
            self.get_logger().error("invalid goal, viewpoints is empty")
            result.success = False
            goal_handle.abort()
            self.busy = 0
            return result

        self.get_logger().info("goal accepted")
        feedback.progress = 0 / 3
        goal_handle.publish_feedback(feedback)

        try:
            grip_vx_bot_at_viewpoint(self.bot, grip_pose, start="open", end="close")
            feedback.progress = 1 / 3
            goal_handle.publish_feedback(feedback)

            move_vx_bot_to_viewpoint(self.bot, show_pose, self.offset, self.wait_duration)
            feedback.progress = 2 / 3
            goal_handle.publish_feedback(feedback)

            grip_vx_bot_at_viewpoint(self.bot, grip_pose, start="close", end="open")
            feedback.progress = 3 / 3
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
        node = GripObjectToShowServer()
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

