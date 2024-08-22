#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rms_msgs.action import PointCloudCollection

import asyncio
from os import path
import sys

sys.path.append(path.abspath(path.join(path.dirname(__file__), "../../../../")))

from modules.collection import (
    get_config_path,
    get_scans_path,
    get_model_path,
    get_viewpoints,
)


class ReconfigurableManufacturingSystems(Node):
    """
    
    """
    def __init__(self):
        """
        
        """
        super().__init__("rms_main")

        self.declare_parameter("manipulators", ["vx250", "vx300s"])
        self.declare_parameter("collection_action", "collect_pointclouds_at_viewpoints")
        self.declare_parameter("model_name", "object_b")

        self.manipulators = self.get_parameter("manipulators").get_parameter_value().string_array_value
        self._collection_action = self.get_parameter("collection_action").get_parameter_value().string_value
        self.model_name = self.get_parameter("model_name").get_parameter_value().string_value

        home_path = path.expanduser("~/")
        rms_path = path.abspath(path.join(home_path, "rms"))
        self.config_path = get_config_path(rms_path)
        self.scans_path = get_scans_path(rms_path)
        self.model_path = get_model_path(rms_path, self.model_name)

        self._collection_clients = {}
        self._collection_results = {}
        self._collection_counter = 0
        self._collection_count = len(self.manipulators)
        for manipulator in self.manipulators:
            self._collection_clients[manipulator] = ActionClient(
                self,
                PointCloudCollection,
                f"/{manipulator}/{self._collection_action}"
            )

        asyncio.run(self._send_collection_goals())

    
    async def _send_collection_goals(self):
        """
        
        """
        send_tasks = []
        for manipulator, client in self._collection_clients.items():
            if not client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error(f"action server for {manipulator} unavailable")
                return

            goal = PointCloudCollection.Goal()
            goal.viewpoints = get_viewpoints(self.config_path, manipulator)
            goal.scans_path = self.scans_path

            future = client.send_goal_async(goal)
            send_tasks.append(self._handle_collection_result(manipulator, future))


    async def _handle_collection_result(self, manipulator, goal_future):
        """
        Handles the result of a goal and updates the counter.
        """
        result_future = await goal_future
        result = await result_future.result()

        self.get_logger().info(f"received result from {manipulator}: {result}")

        self._collection_counter += 1
        if self._collection_counter == self._collection_count:
            self.get_logger().info("done")


def main(args=None):
    """
    
    """
    try:
        rclpy.init(args=args)
        node = ReconfigurableManufacturingSystems()
    except:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
