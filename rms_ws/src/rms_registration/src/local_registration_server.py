#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rms_msgs.srv import PCDRegistration

import copy
import numpy as np
import open3d as o3d
from os import path
import sys

sys.path.append(path.abspath(path.join(path.dirname(__file__), "../../../../")))
from modules.registration import get_pcd_file_paths, get_pcd_files, get_pcd_transforms, filter_by_axis, prepare_for_local_registration, execute_local_registration # type: ignore


class LocalRegistrationServer(Node):
    """
    
    """

    def __init__(self):
        """
        
        
        """

        super().__init__("local_registration")

        self.declare_parameter("voxel_size", 0.01)
        self.voxel_size = self.get_parameter("voxel_size").get_parameter_value().double_value

        self.srv = self.create_service(
            PCDRegistration, "local_registration", self.execute_callback
        )

    
    def execute_callback(self, request, response):
        """
        
        """

        # get scans
        self.get_logger().info("scan retrieval")
        scans = []
        for manipulator in request.manipulators:
            scan_paths = get_pcd_file_paths(manipulator, request.scans_path)
            scans.extend(get_pcd_files(scan_paths))

        # transform scans
        transforms = []
        self.get_logger().info("scan transformation")
        for manipulator in request.manipulators:
            transforms_ = get_pcd_transforms(manipulator, request.scans_path)
            for transform in transforms_:
                transforms.append(transform)
                # transforms.append(np.linalg.inv(transform))

        transformed = []
        for idx, scan in enumerate(scans, start=0):
            temp = copy.deepcopy(scan)
            temp.transform(np.asarray(transforms[idx], dtype=np.float64))
            transformed.append(temp)

        # filter scans
        filtered = []
        self.get_logger().info("scan filtering")
        for scan in transformed:
            # scan = filter_by_axis(scan, 0.1, 0)
            scan = filter_by_axis(scan, 0.8, 2)
            filtered.append(scan)
        scans = filtered

        # register and filterscans
        merged = scans[0]
        unmerged = scans[1:]
        for idx, scan in enumerate(unmerged, start=2):
            self.get_logger().info(f"scan registration ({idx}/{len(unmerged) + 1})")
            scan_down, merged_down = prepare_for_local_registration(merged, scan, self.voxel_size)
            result = execute_local_registration(scan_down, merged_down)
            scan.transform(result.transformation)
            o3d.visualization.draw_geometries([scan, merged])

        return response


def main(args=None):    
    try:
        rclpy.init(args=args)
        node = LocalRegistrationServer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("keyboard interrupt, shutting down ...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
