#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rms_msgs.srv import PCDRegistration

import numpy as np
import open3d as o3d
from os import path
import sys

sys.path.append(path.abspath(path.join(path.dirname(__file__), "../../../../")))
from modules.registration import get_pcd_file_paths, get_pcd_files, filter_out_ground_plane, filter_out_statistical_outliers, filter_out_radial_outlier, filter_by_axis, prepare_for_global_registration, execute_global_registration, execute_fast_global_registration, prepare_for_local_registration, execute_local_registration, get_random_rotation_matrix, count_nearest_neigbors # type: ignore


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

        # model retrieval
        self.get_logger().info("model retrieval")
        model = o3d.io.read_point_cloud(request.model_path)
        # o3d.visualization.draw_geometries([model])

        # get scans
        self.get_logger().info("scan retrieval")
        scans = []
        for manipulator in request.manipulators:
            scan_paths = get_pcd_file_paths(manipulator, request.scans_path)
            scans.extend(get_pcd_files(scan_paths))

        # # transform scans
        # self.get_logger().info("scan transformation")
        # for manipulator in request.manipulators:
        #     transform_path = get_pcd_transforms(manipulator, request.scans_path)

        # filter scans
        filtered = []
        self.get_logger().info("scan filtering")
        for scan in scans:
            scan = filter_by_axis(scan, 0.1, 0)
            scan = filter_by_axis(scan, 0.8, 2)
            scan = filter_out_ground_plane(scan)
            # scan = filter_out_statistical_outliers(scan, nb_neighbors=10, std_ratio=3.0)
            scan = filter_out_radial_outlier(scan, nb_points=30, radius=2*self.voxel_size)
            filtered.append(scan)
            # o3d.visualization.draw_geometries([scan])
        scans = filtered

        # register scans
        merged = o3d.geometry.PointCloud()
        unmerged1 = [scans[0]]
        unmerged2 = [scans[2]]
        unmerged = unmerged1 + unmerged2
        for idx, scan in enumerate(unmerged, start=2):
            self.get_logger().info(f"scan registration ({idx}/{len(unmerged) + 1})")

            scan__ = scan
            cost = 0
            for _ in range(1):
                r_ = get_random_rotation_matrix()
                scan_ = scan__.transform(r_)
                scan_down, model_down, scan_fpfh, model_fpfh = prepare_for_global_registration(scan_, model, self.voxel_size/2.0)
                result_ransac2_ = execute_fast_global_registration(scan_down, model_down, scan_fpfh, model_fpfh, self.voxel_size/2.0)
                scan_ = scan_.transform(result_ransac2_.transformation)
                cost_ = count_nearest_neigbors(scan_, model, self.voxel_size)
                if cost_ > cost:
                    scan = scan_
                    cost = cost_
                    print(cost)
                      
            print(count_nearest_neigbors(scan, model, self.voxel_size))
            o3d.visualization.draw_geometries([scan, model])

            # scan_down, model_down, scan_fpfh, model_fpfh = prepare_for_global_registration(scan, model, self.voxel_size/2.0)
            # result_ransac = execute_global_registration(scan_down, model_down, scan_fpfh, model_fpfh, self.voxel_size/2.0)
            # scan = scan.transform(result_ransac.transformation)
            # print(result_ransac)
            # o3d.visualization.draw_geometries([scan, model])

            scan_down, model_down = prepare_for_local_registration(scan, model, self.voxel_size/2.0)
            result_icp = execute_local_registration(scan_down, model_down)
            scan = scan.transform(result_icp.transformation)
            print(count_nearest_neigbors(scan, model, self.voxel_size))
            o3d.visualization.draw_geometries([scan, model])

            merged = merged + scan

            # o3d.visualization.draw_geometries([scan_down.transform(result.transformation), model_down])
        o3d.visualization.draw_geometries([merged, model])

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
