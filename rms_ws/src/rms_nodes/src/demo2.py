#!/usr/bin/env python3

##################################################################################
# RMS DEMO 2 : N-Manipulator End-to-End Collection/Registration to Reference Model
# Assumptions : 
#   - secondary.launch.py running on all manipulator machines,
#   - primary.launch.py running on primary machine,
#   - test object is rotated and rescaned by all manipulators in new orientation
##################################################################################

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rms_msgs.action import (
    PointCloudCollection,
    PointCloudRegistration,
)

from os import path
import sys
import time

sys.path.append(path.abspath(path.join(path.dirname(__file__), "../../../../")))

from modules.collection import (
    get_config_path,
    get_scans_path,
    get_model_path,
    get_viewpoints,
)


class ReconfigurableManufacturingSystems(Node):
    """
    Main class for the Reconfigurable Manufacturing Systems (RMS).
    """
    def __init__(self):
        """
        Initialize the RMS node.
        """
        super().__init__("main")

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
        self._collection_counter = 0
        self._collection_count = len(self.manipulators)

        for manipulator in self.manipulators:
            self._collection_clients[manipulator] = ActionClient(
                self,
                PointCloudCollection,
                f"/{manipulator}/{self._collection_action}"
            )

        self._registration_client = ActionClient(
            self,
            PointCloudRegistration,
            "/rms/register_pointclouds_to_model"
        )

        self.status = 0
        self._send_collection_goals(1)


    def _send_collection_goals(self, start_index):
        """
        Send goals to all manipulator action servers.
        """
        for manipulator, client in self._collection_clients.items():
            if not client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error(f"action server for {manipulator} unavailable")
                self._collection_counter = self._collection_counter + 1
                return

            goal = PointCloudCollection.Goal()
            goal.viewpoints = get_viewpoints(self.config_path, manipulator)
            goal.scans_path = self.scans_path
            goal.first_index = start_index

            self.get_logger().info(f"sending collection goal to {manipulator}")

            future = client.send_goal_async(goal)
            future.add_done_callback(lambda future, manipulator=manipulator: self._handle_collection_result(manipulator, future))


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
            time.sleep(5)
            self._send_collection_goals(5)
        elif self._collection_counter == 2 * self._collection_count:
            self.get_logger().info("all online manipulators are done, sending registration goal")
            self._send_registration_goal()
    
    
    def _send_registration_goal(self):
        """
        
        """
        goal = PointCloudRegistration.Goal()
        goal.scans_path = self.scans_path
        goal.model_path = self.model_path

        self.get_logger().info("sending registration goal")

        future = self._registration_client.send_goal_async(goal)
        future.add_done_callback(lambda future: self._handle_registration_result(future))

    
    def _handle_registration_result(self, future):
        """
        
        """
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("registration goal rejected")
            return

        self.get_logger().info("registration goal accepted")
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda future: self._handle_registration_completion(future))

    
    def _handle_registration_completion(self, future):
        """
        
        """
        result = future.result().result
        self.get_logger().info(f"registration result: {result.success}")
        rclpy.shutdown()


def main(args=None):
    """
    
    """
    try:
        rclpy.init(args=args)
        node = ReconfigurableManufacturingSystems()
        rclpy.spin(node)
    except:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
