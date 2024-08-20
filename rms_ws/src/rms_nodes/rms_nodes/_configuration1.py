#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rms_msgs.action import PCDCollection
from rms_msgs.srv import PCDRegistration

from os import path
import sys
import threading

sys.path.append(path.abspath(path.join(path.dirname(__file__), "../../../../")))

from modules.manipulators import VXManipulator, get_vx_node_status, check_for_file, get_primary_write_dir
from modules.collection import write_pcd_from_stl


class RMSConfiguration1(Node):
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

        if not get_vx_node_status(self.primary):
            self.get_logger().error(f"{self.primary.robot_model} is offline, please launch the interbotix control node")
            return
        
        self.path2pcd = path.join(path.expanduser("~/rms/pointclouds/models/"), f"{self.part_id}.pcd")
        self.path2stl = path.join(path.expanduser("~/rms/pointclouds/models/"), f"{self.part_id}.stl")
        if not check_for_file(self.path2pcd):
            self.get_logger().error(f"unable to find {self.part_id}.pcd, please check if it is in the models/ subdirectory")
            if check_for_file(self.path2stl):
                write_pcd_from_stl(self.path2stl, self.path2pcd)
            else:
                self.get_logger().error(f"unable to find {self.part_id}.stl, please check if it is in the models/ subdirectory")
                return
            
        self._collection_client_primary = ActionClient(
            self,
            PCDCollection,
            "/vx250/viewpoint_collection",
        )

        self._registration_client = self.create_client(
            PCDRegistration,
            "/global_registration"
        )

        while not self._registration_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info('registration service not available, waiting again...')

        self._result_lock = threading.Lock()
        self._primary_result = None

        self.collect_pointclouds_on_primary()
        

    def collect_pointclouds_on_primary(self):
        """
        Send goal to the primary manipulator.
        """
        goal = PCDCollection.Goal()
        goal.manipulator = self.primary.robot_model
        goal.read_path = path.expanduser("~/rms/config/vx250_viewpoints.txt")

        primary_dir = get_primary_write_dir(1)
        self.write_path = primary_dir
        goal.write_path = primary_dir

        self._collection_client_primary.wait_for_server()
        self._send_goal_future_primary = self._collection_client_primary.send_goal_async(goal)
        self._send_goal_future_primary.add_done_callback(self.goal_response_primary_callback)

    def goal_response_primary_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('vx250 collection goal rejected :(')
            return

        self.get_logger().info('vx250 collection goal accepted :)')
        self._get_result_future_primary = goal_handle.get_result_async()
        self._get_result_future_primary.add_done_callback(self.get_primary_result_callback)

    def get_primary_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('vx250 collection result: {0}'.format(result.success))
        with self._result_lock:
            self._primary_result = result
            if self._primary_result is not None:
                self.print_final_result()

    def register_pointclouds(self):
        """
        Register point clouds by calling the PCDRegistration service.
        """
        request = PCDRegistration.Request()
        request.manipulators = [self.primary.robot_model]
        request.scans_path = self.write_path
        request.model_path = self.path2pcd

        future = self._registration_client.call_async(request)
        
        # Wait for the service call to complete
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info('registration service call succeeded')
            response = future.result()
            # Process the response if needed
            return response
        else:
            self.get_logger().error('registration service call failed')
            return None



    def print_final_result(self):
        """
        Print the final result after the primary manipulator has completed scanning.
        """
        self.get_logger().info("vx250 has completed scanning")

        self.register_pointclouds()
        rclpy.shutdown()


def main(args=None):    
    try:
        rclpy.init(args=args)
        node = RMSConfiguration1()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down ...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
