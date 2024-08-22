#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Point
from rms_msgs.action import PointCloudRegistration
from rms_msgs.msg import PointCloudStamped

from copy import deepcopy
import open3d as o3d
from os import path
import sys

sys.path.append(path.abspath(path.join(path.dirname(__file__), "../../../../")))

from modules.registration import (
    load_model,
    load_scans,
    filter_pointcloud_by_axis,
    filter_pointcloud_by_removing_ground_plane,
    filter_pointcloud_by_removing_radial_outliers,
    filter_pointcloud_by_removing_statistical_outliers,
    prepare_for_global_registration,
    execute_global_registration,
    execute_local_registration,
    get_registration_confidence,
    filter_pointcloud_within_scaled_model_bbox,
    execute_pointcloud_comparison,
)


class PointCloudRegistrationServer(Node):
    """
    
    """
    def __init__(self):
        """
        
        """
        super().__init__("pointcloud_registration")

        self.declare_parameter("voxel_size", 0.005)
        self.declare_parameter("x_min", -0.2)
        self.declare_parameter("x_max", 0.2)
        self.declare_parameter("y_min", -1.0)
        self.declare_parameter("y_max", 1.0)
        self.declare_parameter("z_min", 0.2)
        self.declare_parameter("z_max", 0.6)
        self.declare_parameter("alignment_threshold", 0.99)
        self.declare_parameter("alignment_attempts", 3)
        self.declare_parameter("registration_service", "register_pointclouds_to_model")
        self.declare_parameter("noise_threshold", 0.002)
        self.declare_parameter("defect_threshold", 0.005)

        self.voxel_size = self.get_parameter("voxel_size").get_parameter_value().double_value
        self.x_min = self.get_parameter("x_min").get_parameter_value().double_value
        self.x_max = self.get_parameter("x_max").get_parameter_value().double_value
        self.y_min = self.get_parameter("y_min").get_parameter_value().double_value
        self.y_max = self.get_parameter("y_max").get_parameter_value().double_value
        self.z_min = self.get_parameter("z_min").get_parameter_value().double_value
        self.z_max = self.get_parameter("z_max").get_parameter_value().double_value
        self.alignment_threshold = self.get_parameter("alignment_threshold").get_parameter_value().double_value
        self.alignment_attempts = self.get_parameter("alignment_attempts").get_parameter_value().integer_value
        self._registration_service = self.get_parameter("registration_service").get_parameter_value().string_value
        self.noise_threshold = self.get_parameter("noise_threshold").get_parameter_value().double_value
        self.defect_threshold = self.get_parameter("defect_threshold").get_parameter_value().double_value

        self._registration_server = ActionServer(
            self,
            PointCloudRegistration, 
            self._registration_service, 
            self._registration_execute
        )

        self.get_logger().info("ready to register pointclouds")

    
    def _registration_execute(self, goal_handle):
        """
        
        """
        goal = goal_handle.request
        feedback = PointCloudRegistration.Feedback()
        result = PointCloudRegistration.Result()

        if not goal.scans_path or not goal.model_path:
            self.get_logger().error("invalid goal: scans_file_path or model_file_path is empty")
            result.success = False
            goal_handle.abort(result)
            return
        
        model = load_model(goal.model_path)
        scans = load_scans(goal.scans_path)
        registration = o3d.geometry.PointCloud()
        feedback.successes = 0
        feedback.failures = 0
        counter = 1

        for scan in scans:
            scan = self.prefilter_pointcloud(scan)
            o3d.visualization.draw_geometries([scan])

            attempts = 0
            confidence = 0
            while attempts < self.alignment_attempts and confidence < self.alignment_threshold:
                scan = self.register_pointcloud(scan, model)
                confidence = get_registration_confidence(scan, model, self.voxel_size)
                attempts += 1
                self.get_logger().info(f"scan #{counter:02d} ({attempts}/{self.alignment_attempts}) : currently {confidence:.4f}% confident in alignment")

            if confidence < self.alignment_threshold:
                feedback.failures = feedback.failures + 1
                goal_handle.publish_feedback(feedback)
            elif confidence >= self.alignment_threshold:
                registration = registration + scan
                feedback.successes = feedback.successes + 1
                goal_handle.publish_feedback(feedback)

            counter += 1

        self.get_logger().info(f"registration success rate : {feedback.successes / len(scans)}")
        
        registration = filter_pointcloud_by_removing_statistical_outliers(registration, 50, 1.0)
        registration = filter_pointcloud_within_scaled_model_bbox(registration, model, 1.2)

        likely_scan_with_noise, likely_defects_p = execute_pointcloud_comparison(registration, model, self.defect_threshold)
        likely_scan_without_noise, likely_noise = execute_pointcloud_comparison(likely_scan_with_noise, model, self.noise_threshold)
        likely_scan, likely_defects_m = execute_pointcloud_comparison(model, likely_scan_with_noise, self.noise_threshold)
        
        self.visualize_pointcloud(model, likely_scan, likely_noise, likely_defects_p, likely_defects_m)

        file_path = path.abspath(path.join(goal.scans_path, "registration.pcd"))
        o3d.io.write_point_cloud(file_path, registration)

        goal_handle.succeed()
        result.success = True
        return result


    def prefilter_pointcloud(self, scan):
        """
        
        """
        scan = filter_pointcloud_by_axis(scan, self.x_min, self.x_max, axis=0)
        scan = filter_pointcloud_by_axis(scan, self.y_min, self.y_max, axis=1)
        scan = filter_pointcloud_by_axis(scan, self.z_min, self.z_max, axis=2)
        scan = filter_pointcloud_by_removing_ground_plane(scan)
        scan = filter_pointcloud_by_removing_radial_outliers(scan)
        scan = filter_pointcloud_by_removing_statistical_outliers(scan, nb_neighbors=30, std_ratio=1.0)
        return scan
    

    def register_pointcloud(self, scan, model):
        """
        
        """
        scan_down, model_down, scan_fpfh, model_fpfh = prepare_for_global_registration(scan, model, self.voxel_size)
        result_global = execute_global_registration(scan_down, model_down, scan_fpfh, model_fpfh, self.voxel_size)
        scan.transform(result_global.transformation)
        result_local = execute_local_registration(scan, model, self.voxel_size)
        scan.transform(result_local.transformation)
        return scan


    def visualize_pointcloud(self, model, likely_scan, likely_noise, likely_defects_p, likely_defects_m):
        """
        
        """
        model_temp = deepcopy(model)
        scan_temp = deepcopy(likely_scan)
        noise_temp = deepcopy(likely_noise)
        defects_p_temp = deepcopy(likely_defects_p)
        defects_m_temp = deepcopy(likely_defects_m)

        model_temp.paint_uniform_color([0.0000, 0.4470, 0.7410])
        scan_temp.paint_uniform_color([0.8500, 0.3250, 0.0980])
        noise_temp.paint_uniform_color([0.9290, 0.6940, 0.1250])
        defects_p_temp.paint_uniform_color([0.6350, 0.0780, 0.1840])
        defects_m_temp.paint_uniform_color([0.4940, 0.1840, 0.5560])

        o3d.visualization.draw_geometries([model_temp, scan_temp, noise_temp, defects_p_temp, defects_m_temp])


def main(args=None):
    """
    
    """
    try:
        rclpy.init(args=args)
        node = PointCloudRegistrationServer()
        rclpy.spin(node)
    except:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
