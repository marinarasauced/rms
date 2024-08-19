#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rms_msgs.action import PCDCollection

from os import path
import sys
import threading

sys.path.append(path.abspath(path.join(path.dirname(__file__), "../../../../")))

from modules.manipulators import VXManipulator, get_vx_node_status, check_for_file, get_primary_write_dir
from modules.collection import write_pcd_from_stl


class RMSConfiguration2(Node):
    """
    Node for RMS Configuration with two manipulators running synchronously.
    """
    def __init__(self):
        """
        Initialize the node, setup the manipulators and start the point cloud collection process.
        """
        super().__init__("rms_configuration1")
        self.declare_parameter("part_id", "test")
        self.part_id = self.get_parameter("part_id").get_parameter_value().string_value

        self.primary = VXManipulator("primary")
        self.secondary = VXManipulator("secondary")

        if not get_vx_node_status(self.primary):
            self.get_logger().error(f"{self.primary.robot_model} is offline, please launch the interbotix control node")
            return

        if not get_vx_node_status(self.secondary):
            self.get_logger().error(f"{self.secondary.robot_model} is offline, please launch the interbotix control node")
            return
        
        path2pcd = path.join(path.expanduser("~/rms/pointclouds/models/"), f"{self.part_id}.pcd")
        path2stl = path.join(path.expanduser("~/rms/pointclouds/models/"), f"{self.part_id}.stl")
        if not check_for_file(path2pcd):
            self.get_logger().error(f"unable to find {self.part_id}.pcd, please check if it is in the models/ subdirectory")
            if check_for_file(path2stl):
                write_pcd_from_stl(path2stl, path2pcd)
            else:
                self.get_logger().error(f"unable to find {self.part_id}.stl, please check if it is in the models/ subdirectory")
                return
            
        self._collection_client_primary = ActionClient(
            self,
            PCDCollection,
            "/vx250/viewpoint_collection",
        )
        self._collection_client_secondary = ActionClient(
            self,
            PCDCollection,
            "/vx250/secondary_viewpoint_collection",
        )

        self._result_lock = threading.Lock()
        self._primary_result = None
        self._secondary_result = None

        self.collect_pointclouds_on_primary()
        self.collect_pointclouds_on_secondary()

    def collect_pointclouds_on_primary(self):
        """
        Send goal to the primary manipulator.
        """
        goal = PCDCollection.Goal()
        goal.manipulator = self.primary.robot_model
        goal.read_path = path.expanduser("~/rms/config/vx250_viewpoints.txt")

        primary_dir = get_primary_write_dir(1)
        goal.write_path = primary_dir

        self._collection_client_primary.wait_for_server()
        self._send_goal_future_primary = self._collection_client_primary.send_goal_async(goal)
        self._send_goal_future_primary.add_done_callback(self.goal_response_primary_callback)

    def goal_response_primary_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Primary goal rejected :(')
            return

        self.get_logger().info('Primary goal accepted :)')
        self._get_result_future_primary = goal_handle.get_result_async()
        self._get_result_future_primary.add_done_callback(self.get_primary_result_callback)

    def get_primary_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Primary result: {0}'.format(result.success))
        with self._result_lock:
            self._primary_result = result
            if self._secondary_result is not None:
                self.print_final_result()

    def collect_pointclouds_on_secondary(self):
        """
        Send goal to the secondary manipulator.
        """
        goal = PCDCollection.Goal()
        goal.manipulator = self.secondary.robot_model
        goal.read_path = path.expanduser("~/rms/config/vx250_secondary_viewpoints.txt")

        secondary_dir = get_primary_write_dir(2)
        goal.write_path = secondary_dir

        self._collection_client_secondary.wait_for_server()
        self._send_goal_future_secondary = self._collection_client_secondary.send_goal_async(goal)
        self._send_goal_future_secondary.add_done_callback(self.goal_response_secondary_callback)

    def goal_response_secondary_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Secondary goal rejected :(')
            return

        self.get_logger().info('Secondary goal accepted :)')
        self._get_result_future_secondary = goal_handle.get_result_async()
        self._get_result_future_secondary.add_done_callback(self.get_secondary_result_callback)

    def get_secondary_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Secondary result: {0}'.format(result.success))
        with self._result_lock:
            self._secondary_result = result
            if self._primary_result is not None:
                self.print_final_result()

    def print_final_result(self):
        """
        Print the final result after both primary and secondary goals are completed.
        """
        self.get_logger().info("Both manipulators have completed their actions.")
        rclpy.shutdown()


def main(args=None):    
    try:
        rclpy.init(args=args)
        node = RMSConfiguration2()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down ...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
