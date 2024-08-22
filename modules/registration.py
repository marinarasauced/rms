
from glob import glob
import numpy as np
import open3d as o3d
from os import path


def load_pointcloud(file_path):
    """
    
    """
    file_data = o3d.io.read_point_cloud(file_path)
    return file_data


def load_model(model_path):
    """
    
    """
    model = load_pointcloud(model_path)
    return model


def load_scans(scans_path):
    """
    
    """
    search_path = path.join(scans_path, "vx*_*.pcd")
    scan_paths = glob(search_path)
    scans = [load_pointcloud(scan_path) for scan_path in scan_paths]
    return scans


def filter_pointcloud_by_axis(pcd, min_value, max_value, axis):
    """

    """
    array = np.asarray(pcd.points)
    indices = np.where((array[:, axis] >= min_value) & (array[:, axis] <= max_value))
    filtered = pcd.select_by_index(indices[0])
    return filtered


def filter_pointcloud_by_removing_ground_plane(pcd,  distance_threshold=0.01, ransac_n=3, num_iterations=10000):
    """
    
    """
    _, inliers = pcd.segment_plane(distance_threshold, ransac_n, num_iterations)
    inlier_cloud = pcd.select_by_index(inliers, invert=True)
    return inlier_cloud


def filter_pointcloud_by_removing_radial_outliers(pcd, nb_points=16, radius=0.01):
    """
    
    """
    _, inliers = pcd.remove_radius_outlier(nb_points, radius)
    inlier_cloud = pcd.select_by_index(inliers)
    return inlier_cloud


def filter_pointcloud_by_removing_statistical_outliers(pcd, nb_neighbors=20, std_ratio=3.0):
    """

    """
    _, inliers = pcd.remove_statistical_outlier(nb_neighbors, std_ratio)
    inlier_cloud = pcd.select_by_index(inliers)
    return inlier_cloud


def preprocess_point_cloud(pcd, voxel_size):
    """
    
    """
    pcd_down = pcd.voxel_down_sample(voxel_size)
    radius_normal = voxel_size * 2
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(
            radius=radius_normal, max_nn=30
        )
    )
    radius_feature = voxel_size * 10
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(
            radius=radius_feature, max_nn=100
        )
    )
    return pcd_down, pcd_fpfh


def prepare_for_global_registration(scan, model, voxel_size):
    """
    
    """

    scan_down, scan_fpfh = preprocess_point_cloud(scan, voxel_size)
    model_down, model_fpfh = preprocess_point_cloud(model, voxel_size)
    return scan_down, model_down, scan_fpfh, model_fpfh


def execute_global_registration(scan_down, model_down, scan_fpfh, model_fpfh, voxel_size):
    """
    
    """
    distance_threshold = voxel_size * 1.5
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        scan_down,
        model_down,
        scan_fpfh,
        model_fpfh,
        False,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, 
        [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.5),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], 
        o3d.pipelines.registration.RANSACConvergenceCriteria(1000000, 10000))
    return result


def execute_local_registration(scan_down, model_down, voxel_size):
    """
    
    """
    distance_threshold = voxel_size * 1
    result = o3d.pipelines.registration.registration_icp(
        scan_down,
        model_down,
        distance_threshold,
        np.identity(4),
        o3d.pipelines.registration.TransformationEstimationPointToPlane()
    )
    return result

def execute_pointcloud_comparison(source, target, distance_threshold=0.0025):
    """
    
    """
    target_kd_tree = o3d.geometry.KDTreeFlann(target)
    likely_source = []
    likely_noise = []
    for point in source.points:
        points = np.asarray(point)
        [k, _, _] = target_kd_tree.search_radius_vector_3d(points, distance_threshold)
        if k > 0:
            likely_source.append(point)
        else:
            likely_noise.append(point)
    likely_source_ = o3d.geometry.PointCloud()
    likely_noise_ = o3d.geometry.PointCloud()
    if likely_source:
        likely_source_.points = o3d.utility.Vector3dVector(likely_source)
    if likely_noise:
        likely_noise_.points = o3d.utility.Vector3dVector(likely_noise)
    return likely_source_, likely_noise_


def get_registration_confidence(scan, model, distance_threshold):
    """
    
    """
    likely_scan, _ = execute_pointcloud_comparison(scan, model, distance_threshold)
    registration_confidence = len(likely_scan.points) / len(scan.points)
    return registration_confidence


def filter_pointcloud_within_scaled_model_bbox(source, target, scale):
    """

    """
    
    bbox = target.get_axis_aligned_bounding_box()
    center = bbox.get_center()
    bbox_size = bbox.get_extent()
    scaled_min_bound = center - (bbox_size * scale / 2)
    scaled_max_bound = center + (bbox_size * scale / 2)
    scaled_min_bound = np.asarray(scaled_min_bound)
    scaled_max_bound = np.asarray(scaled_max_bound)
    
    filtered_points = []
    for point in source.points:
        point_array = np.asarray(point)
        if np.all(point_array >= scaled_min_bound) and np.all(point_array <= scaled_max_bound):
            filtered_points.append(point_array)
    
    filtered_pointcloud = o3d.geometry.PointCloud()
    if filtered_points:
        filtered_pointcloud.points = o3d.utility.Vector3dVector(np.array(filtered_points))
    
    return filtered_pointcloud