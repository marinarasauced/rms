
import copy
import glob
import numpy as np
import open3d as o3d
from os import path


def get_pcd_file(read_path):
    """
    
    """
    read_file = o3d.io.read_point_cloud(read_path)
    return read_file


def get_pcd_files(manipulator, read_path):
    """
    
    """
    search_path = path.join(read_path, f"{manipulator}_*.pcd")
    read_paths = glob.glob(search_path)
    read_files = [o3d.io.read_point_cloud(path) for path in read_paths]
    return read_files


def filter_pcd_by_axis(pcd, min_value, max_value, axis):
    """
    Filters the input point cloud by retaining only the points whose coordinates along the specified axis
    fall within the range [min_value, max_value].

    Parameters:
    -----------
    pcd : open3d.geometry.PointCloud
        The input point cloud to be filtered.
    min_value : float
        The minimum value along the specified axis for the points to be retained.
    max_value : float
        The maximum value along the specified axis for the points to be retained.
    axis : int
        The axis along which to filter the points (0 for x-axis, 1 for y-axis, 2 for z-axis).

    Returns:
    --------
    filtered : open3d.geometry.PointCloud
        The filtered point cloud with points only within the specified range along the given axis.
    """
    array = np.asarray(pcd.points)
    indices = np.where((array[:, axis] >= min_value) & (array[:, axis] <= max_value))
    filtered = pcd.select_by_index(indices[0])
    return filtered


def filter_pcd_by_removing_ground_plane(pcd,  distance_threshold=0.01, ransac_n=3, num_iterations=10000):
    """
    
    """
    _, inliers = pcd.segment_plane(distance_threshold, ransac_n, num_iterations)
    inlier_cloud = pcd.select_by_index(inliers, invert=True)
    return inlier_cloud


def filter_pcd_by_removing_radial_outliers(pcd, nb_points=16, radius=0.01):
    """
    
    """
    _, inliers = pcd.remove_radius_outlier(nb_points, radius)
    inlier_cloud = pcd.select_by_index(inliers)
    return inlier_cloud


def filter_pcd_by_removing_statistical_outliers(pcd, nb_neighbors=20, std_ratio=3.0):
    """

    """
    _, inliers = pcd.remove_statistical_outlier(nb_neighbors, std_ratio)
    inlier_cloud = pcd.select_by_index(inliers)
    return inlier_cloud


def estimate_point_cloud_normals(pcd, voxel_size):
    """
    
    """
    pcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(
            radius=voxel_size * 2,
            max_nn=30
        )
    )
    return pcd


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
    radius_feature = voxel_size * 5
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
        True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, 
        [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.5),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], 
        o3d.pipelines.registration.RANSACConvergenceCriteria(1000000, 1000))
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



def filter_points_in_scaled_bbox(pcd1, pcd2, scale_factor):
    """
    Filters points in pcd1 that fall within the scaled bounding box of pcd2.
    
    Parameters:
    - pcd1: The first point cloud (o3d.geometry.PointCloud).
    - pcd2: The second point cloud (o3d.geometry.PointCloud).
    - scale_factor: The factor by which to scale the bounding box of pcd2.
    
    Returns:
    - A new point cloud containing all points in pcd1 within the scaled bounding box of pcd2.
    """
    
    # Get the axis-aligned bounding box of pcd2
    bbox = pcd2.get_axis_aligned_bounding_box()
    
    # Calculate the center of the bounding box
    center = bbox.get_center()
    
    # Calculate the size of the bounding box (range in each dimension)
    bbox_size = bbox.get_extent()
    
    # Scale the bounding box size
    scaled_min_bound = center - (bbox_size * scale_factor / 2)
    scaled_max_bound = center + (bbox_size * scale_factor / 2)
    
    # Convert bounds to numpy arrays for element-wise comparison
    scaled_min_bound = np.asarray(scaled_min_bound)
    scaled_max_bound = np.asarray(scaled_max_bound)
    
    # Filter points in pcd1 that are within the scaled bounding box
    filtered_points = []
    for point in pcd1.points:
        point_array = np.asarray(point)
        if np.all(point_array >= scaled_min_bound) and np.all(point_array <= scaled_max_bound):
            filtered_points.append(point_array)
    
    # Create a new point cloud from the filtered points
    filtered_pcd = o3d.geometry.PointCloud()
    if filtered_points:
        filtered_pcd.points = o3d.utility.Vector3dVector(np.array(filtered_points))
    
    return filtered_pcd


def get_pcd_differences(source, target, distance_threshold):
    """
    Identify and return the points in source that DO NOT have at least one neighbor in the target point cloud within a given distance threshold.
    """
    target_kd_tree = o3d.geometry.KDTreeFlann(target)
    difference_pcd = []
    match_pcd = []
    for point in source.points:
        points = np.asarray(point)
        [k, _, _] = target_kd_tree.search_radius_vector_3d(points, distance_threshold)
        if k > 0:
            match_pcd.append(point)
        else:
            difference_pcd.append(point)
    difference_pcd_ = o3d.geometry.PointCloud()
    match_pcd_ = o3d.geometry.PointCloud()
    if difference_pcd:
        difference_pcd_.points = o3d.utility.Vector3dVector(difference_pcd)
    if match_pcd:
        match_pcd_.points = o3d.utility.Vector3dVector(match_pcd)
    return match_pcd_, difference_pcd_


def visualize_pcd(pcd):
    """
    
    """
    # pcd_ = copy.deepcopy(pcd)
    # pcd_.paint_uniform_color([0.5, 0.5, 0.5])
    # o3d.visualization.draw_geometries([pcd_])
    o3d.visualization.draw_geometries([pcd])