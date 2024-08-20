#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rms_msgs.srv import PCDRegistration

import copy
import open3d as o3d
from os import path
import sys

sys.path.append(path.abspath(path.join(path.dirname(__file__), "../../../../")))

from modules.registration import *


class GlobalRegistrationServer(Node):
    """
    
    """
    def __init__(self):
        """
        
        """
        super().__init__("global_registration_server")

        self.declare_parameter("voxel_size", 0.005)
        self.declare_parameter("x_min", -0.2)
        self.declare_parameter("x_max", 0.2)
        self.declare_parameter("y_min", -1.0)
        self.declare_parameter("y_max", 1.0)
        self.declare_parameter("z_min", 0.2)
        self.declare_parameter("z_max", 0.6)
        self.declare_parameter("alignment_threshold", 0.99)

        self.voxel_size = self.get_parameter("voxel_size").get_parameter_value().double_value
        self.x_min = self.get_parameter("x_min").get_parameter_value().double_value
        self.x_max = self.get_parameter("x_max").get_parameter_value().double_value
        self.y_min = self.get_parameter("y_min").get_parameter_value().double_value
        self.y_max = self.get_parameter("y_max").get_parameter_value().double_value
        self.z_min = self.get_parameter("z_min").get_parameter_value().double_value
        self.z_max = self.get_parameter("z_max").get_parameter_value().double_value
        self.alignment_threshold = self.get_parameter("alignment_threshold").get_parameter_value().double_value

        self._registration_server = self.create_service(
            PCDRegistration, 
            "global_registration", 
            self._execute_callback
        )


    def _execute_callback(self, request, response):
        """
        
        """
        model = get_pcd_file(request.model_path)
        scans = []
        for manipulator in request.manipulators:
            scans = scans + get_pcd_files(manipulator, request.scans_path)
        registration = o3d.geometry.PointCloud()

        registration_counter = 0
        for scan in scans:

            scan = filter_pcd_by_axis(scan, self.x_min, self.x_max, axis=0)
            scan = filter_pcd_by_axis(scan, self.y_min, self.y_max, axis=1)
            scan = filter_pcd_by_axis(scan, self.z_min, self.z_max, axis=2)
            scan = filter_pcd_by_removing_ground_plane(scan)
            scan = filter_pcd_by_removing_radial_outliers(scan)
            scan = filter_pcd_by_removing_statistical_outliers(scan, nb_neighbors=30, std_ratio=1.0)
            # scan_ = copy.deepcopy(scan)

            scan = self.register_pcds(scan, model)

            alignment_metric = get_alignment_confidence(scan, model, self.voxel_size)
            if alignment_metric < self.alignment_threshold:
                self.get_logger().info(f"alignment metric {alignment_metric} < {self.alignment_threshold} cutoff, retrying alignment")

                scan = self.register_pcds(scan, model)
                realignment_metric = get_alignment_confidence(scan, model, self.voxel_size)
                if realignment_metric < self.alignment_threshold:
                    self.get_logger().info(f"realignment metric {realignment_metric} < {self.alignment_threshold} cutoff")
                else:
                    self.get_logger().info(f"realignment metric {realignment_metric} >= {self.alignment_threshold} cutoff")
                    registration = registration + scan
                    registration_counter += 1
            else:
                self.get_logger().info(f"alignment metric {alignment_metric} >= {self.alignment_threshold} cutoff")
                registration = registration + scan
                registration_counter += 1

        registration = filter_pcd_by_removing_statistical_outliers(registration, 50, 1.0)
        registration_counter /= len(scans)
        self.get_logger().info(f"registration success rate: {registration_counter}")

        merged_without_model_ = filter_points_in_scaled_bbox(registration, model, 1.1)
        self.visualize_pcds(merged_without_model_, model)

        # FILTER OUT POTENTIAL NOISE OF EXTERNAL DEFECTS (DOES NOT ACCOUNT FOR DENTS AND IS BAD BC MAY REMOVE ACTUAL EXTERNAL DEFECTS)
        potential_true1, potential_defects1 = get_pcd_differences(merged_without_model_, model)
        self.visualize_pcds_with_potential_defects(potential_true1, model, potential_defects1)
        self.visualize_pcd(potential_true1)

        
        # FILTER BY DETECTING WHAT PARTS OF THE MODEL HAVE NOT BEEN REGISTERED IN THE COMBINED SCAN
        potential_true2, potential_defects2 = get_pcd_differences(model, merged_without_model_)
        self.visualize_pcds_with_potential_defects(potential_true2, model, potential_defects2)



        return response
    

    def register_pcds(self, scan, model):
        """
        
        """
        scan_down, model_down, scan_fpfh, model_fpfh = prepare_for_global_registration(scan, model, self.voxel_size)
        result_global = execute_global_registration(scan_down, model_down, scan_fpfh, model_fpfh, self.voxel_size)
        scan.transform(result_global.transformation)

        result_local = execute_local_registration(scan, model, self.voxel_size)
        scan.transform(result_local.transformation)

        return scan

    def visualize_pcd(self, scan):
        """
        
        """
        o3d.visualization.draw_geometries([scan])


    def visualize_pcds(self, scan, model):
        """
        
        """
        scan_temp = copy.deepcopy(scan)
        model_temp = copy.deepcopy(model)
        scan_temp.paint_uniform_color([0.9290, 0.6940, 0.1250])
        model_temp.paint_uniform_color([0, 0.4470, 0.7410])
        o3d.visualization.draw_geometries([scan_temp, model_temp])


    def visualize_pcds_with_potential_defects(self, scan, model, noise):
        """
        
        """
        scan_temp = copy.deepcopy(scan)
        model_temp = copy.deepcopy(model)
        noise_temp = copy.deepcopy(noise)
        scan_temp.paint_uniform_color([0.9290, 0.6940, 0.1250])
        model_temp.paint_uniform_color([0, 0.4470, 0.7410])
        noise_temp.paint_uniform_color([0.8500, 0.3250, 0.0980])
        o3d.visualization.draw_geometries([scan_temp, model_temp, noise_temp])


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
