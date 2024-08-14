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

from modules.registration2 import *


class GlobalRegistrationServer(Node):
    """
    
    """
    def __init__(self):
        """
        
        """
        super().__init__("global_registration_server")
        self.declare_parameter("voxel_size", 0.0075)
        self.voxel_size = self.get_parameter("voxel_size").get_parameter_value().double_value
        self.srv = self.create_service(PCDRegistration, "global_registration", self.execute_callback)


    def execute_callback(self, request, response):
        """
        
        """
        # retrieve model and scans
        model = get_pcd_file(request.model_path)
        vx250_scans = get_pcd_files("vx250", request.scans_path)
        vx300s_scans = get_pcd_files("vx300s", request.scans_path)

        scans = vx250_scans + vx300s_scans
        # scans = [scans[0]] # FOR TESTING ONLY

        # filter scans to extract object fragments
        merged_with_model = model
        merged_without_model = o3d.geometry.PointCloud()
        for scan in scans:
            scan = filter_pcd_by_axis(scan, min_value=-0.2, max_value=0.2, axis=0)
            scan = filter_pcd_by_axis(scan, min_value=0.2, max_value=0.6, axis=2)
            scan = filter_pcd_by_removing_ground_plane(scan)
            scan = filter_pcd_by_removing_radial_outliers(scan)
        
            # globally register object fragments to model
            scan_down, model_down, scan_fpfh, model_fpfh = prepare_for_global_registration(scan, model, self.voxel_size)

            result_global = execute_global_registration(scan_down, model_down, scan_fpfh, model_fpfh, self.voxel_size)
            global_tf = result_global.transformation

            scan = scan.transform(global_tf)
            scan_down = scan_down.transform(global_tf)

            # locally register object fragments to model
            result_local = execute_local_registration(scan, model, self.voxel_size)
            local_tf = result_local.transformation

            scan = scan.transform(local_tf)
            scan_down = scan_down.transform(local_tf)

            #
            self.visualize_pcds(scan, model)
            merged_with_model = merged_with_model + scan
            merged_without_model = merged_without_model + scan

        visualize_pcd(merged_without_model)
        self.visualize_pcds(merged_without_model, model)

        return response
    

    def visualize_pcds(self, scan, model):
        """
        
        """
        scan_temp = copy.deepcopy(scan)
        model_temp = copy.deepcopy(model)
        scan_temp.paint_uniform_color([1, 0.706, 0])
        model_temp.paint_uniform_color([0, 0.651, 0.929])
        o3d.visualization.draw_geometries([scan_temp, model_temp])


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
