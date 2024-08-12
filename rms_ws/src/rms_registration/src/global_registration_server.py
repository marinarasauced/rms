#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rms_msgs.srv import PCDRegistration

import open3d as o3d
from os import path
import sys

sys.path.append(path.abspath(path.join(path.dirname(__file__), "../../../../")))
from modules.registration import get_pcd_file_paths, get_pcd_files, get_pcd_transforms, filter_by_axis, prepare_for_global_registration, execute_global_registration # type: ignore


class GlobalRegistrationServer(Node):
    """
    
    """

    def __init__(self):
        """
        
        
        """

        super().__init__("global_registration")

        self.declare_parameter("voxel_size", 0.01)
        self.voxel_size = self.get_parameter("voxel_size").get_parameter_value().double_value

        self.srv = self.create_service(
            PCDRegistration, "global_registration", self.execute_callback
        )

    
    def execute_callback(self, request, response):
        """
        
        """

        # get scans
        self.get_logger().info("scan retrieval")
        scans = []
        for path in request.scan:
            scans.extend(get_pcd_files(request.scan_paths))

        # # transform scans
        # self.get_logger().info("scan transformation")
        # for manipulator in request.manipulators:
        #     transform_path = get_pcd_transforms(manipulator, request.scans_path)

        # # filter scans
        # filtered = []
        # self.get_logger().info("scan filtering")
        # for scan in scans:
        #     scan = filter_by_axis(scan, 0.1, 0)
        #     scan = filter_by_axis(scan, 0.8, 2)
        #     filtered.append(scan)

        # register scans
        merged = scans[0]
        unmerged = scans[1:]
        for idx, scan in enumerate(unmerged, start=2):
            self.get_logger().info(f"scan registration ({idx}/{len(unmerged) + 1})")
            scan_down, merged_down, scan_fpfh, merged_fpfh = prepare_for_global_registration(merged, scan, self.voxel_size)
            result = execute_global_registration(scan_down, merged_down, scan_fpfh, merged_fpfh, self.voxel_size)
            scan.transform(result.transformation)
            o3d.visualization.draw_geometries([scan, merged])


        return response


def main(args=None):    
    try:
        rclpy.init(args=args)
        node = GlobalRegistrationServer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("keyboard interrupt, shutting down ...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
