#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rms_msgs.action import (
    GripObjectToScan,
    PointCloudCollection,
    PointCloudRegistration,
)
from rms_msgs.msg import ViewPoint

from os import path
import sys

sys.path.append(path.abspath(path.join(path.dirname(__file__), "../../../../")))

from modules.collection import (
    get_config_path,
    get_scans_path,
    get_viewpoints,
)


class ReconfigurableManufacturingSystemsGripDemo(Node):
    """
    Main class for the Reconfigurable Manufacturing Systems (RMS).
    """
    def __init__(self):
        """
        Initialize the RMS node.
        """
        super().__init__("main")

        
        self.declare_parameter("collection_action", "collect_pointclouds_at_viewpoints")
        self.declare_parameter("grip_action", "grip_object_to_scan")

        self._collection_action = self.get_parameter("collection_action").get_parameter_value().string_value
        self._grip_action = self.get_parameter("grip_action").get_parameter_value().string_value

        home_path = path.expanduser("~/")
        rms_path = path.abspath(path.join(home_path, "rms"))
        self.config_path = get_config_path(rms_path)
        self.scans_path = get_scans_path(rms_path)

        self._grip_client = ActionClient(
            self,
            GripObjectToScan,
            f"/vx300s/{self._grip_action}"
        )

        self._action_client = ActionClient(
            self,
            PointCloudCollection,
            f"/vx250/{self._collection_action}"
        )

        self._grip_goal = GripObjectToScan.Goal()

        self._grip_goal.viewpoints.append(ViewPoint())
        self._grip_goal.viewpoints[0].position.x = 0.5
        self._grip_goal.viewpoints[0].position.y = 0.0
        self._grip_goal.viewpoints[0].position.z = 0.2

        self._grip_goal.viewpoints.append(ViewPoint())
        self._grip_goal.viewpoints[1].position.x = 0.45
        self._grip_goal.viewpoints[1].position.y = 0.0
        self._grip_goal.viewpoints[1].position.z = 0.23


        self._send_collection_goals()


    def _send_collection_goals(self):
        """
        Send goals to all manipulator action servers.
        """
        _grip_future = self._grip_client.send_goal_async(self._grip_goal)
        _grip_future.add_done_callback(lambda future: self._handle_collection_result(future))


    def _handle_collection_result(self, manipulator, future):
        """
        Handles the result of a goal and updates the counter.
        """
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error(f"collection goal rejected by {manipulator}")
            return

        self.get_logger().info(f"collection goal accepted by {manipulator}")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda future, manipulator=manipulator: self._handle_collection_completion(manipulator, future))


    def _handle_collection_completion(self, manipulator, future):
        """
        Finalizes the collection process and checks if all manipulators are done.
        """
        result = future.result().result
        self.get_logger().info(f"received collection result from {manipulator}: {result.success}")

        self._collection_counter += 1

        if self._collection_counter == self._collection_count:
            self.get_logger().info("all online manipulators are done, sending registration goal")
            rclpy.shutdown()


def main(args=None):
    """
    
    """
    try:
        rclpy.init(args=args)
        node = ReconfigurableManufacturingSystemsGripDemo()
        rclpy.spin(node)
    except:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
