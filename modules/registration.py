
import copy
import glob
import open3d as o3d
from os import path
import numpy as np
from scipy.spatial.transform import Rotation as R



def get_pcd_file_paths(manipulator, read_path):
    """
    Get the absolute paths to all PCD scans in a directory for a given manipulator.

    Args:
        manipulator (string): The abbreviated name of the manipulator; e.g., vx250 or vx300s
        read_path (string): The absolute path of the directory containing the PCD scans.

    Returns:
        list[str]: A list of the absolute paths of the PCD scans in the read_path.
    """

    search_path = path.join(read_path, f"{manipulator}_*.pcd")
    search_files = []
    search_files.extend(glob.glob(search_path))
    return search_files


def get_pcd_files(read_paths):
    """
    Get the file content of all PCD scans in the absolute file paths.

    Args:
        read_paths (list[str]): A list of the absolute paths of PCD scans.

    Returns:
        list [o3d.geometry.PointCloud]: A list of the file contents of the PCD files in the absolute file paths.
    """

    pcd_files = [o3d.io.read_point_cloud(path) for path in read_paths]
    return pcd_files


def get_pcd_transforms(manipulator, read_path):
    """
    Get the transforms of the manipulator base to the location of the PCD scans for the scans in the read path.

    Args:
        manipulator (string): The abbreviated name of the manipulator; e.g., vx250 or vx300s
        read_path (string): The absolute path of the directory containing the PCD scans.

    Returns:
        
    """

    search_path = path.join(read_path, f"{manipulator}_viewpoints.txt")
    search_file = []
    search_file.extend(glob.glob(search_path))

    transforms = []
    with open(search_file[0], "r") as file:
        lines = file.readlines()
        lines_ = [np.fromstring(line.strip(), sep=",") for line in lines]
        for line in lines_:
            translation = np.array([line[1], line[2], line[3]])
            rotation = np.array([line[4], line[5], line[6], line[7]])
            transform = np.identity(4)
            transform[0:3, 3] = translation
            transform[0:3, 0:3] = o3d.geometry.get_rotation_matrix_from_quaternion(rotation)
            transforms.append(transform)
    return transforms


def filter_by_axis(pcd, threshold, axis):
    """
    
    """


    array = np.asarray(pcd.points)
    indices = np.where(np.abs(array[:, axis]) < threshold)
    filtered = pcd.select_by_index(indices[0])
    return filtered


def filter_out_ground_plane(pcd, threshold=0.005, ransac_n=3, iterations=10000):
    """
    
    """

    plane_model, inliers = pcd.segment_plane(
        distance_threshold=threshold,
        ransac_n=ransac_n,
        num_iterations=iterations
    )
    # [a, b, c, d] = plane_model
    # ground_plane = pcd.select_by_index(inliers)
    non_ground_plane = pcd.select_by_index(inliers, invert=True)
    return non_ground_plane


def filter_out_statistical_outliers(pcd, nb_neighbors=20, std_ratio=2.0):
    """
    
    """

    _, ind = pcd.remove_statistical_outlier(nb_neighbors, std_ratio)
    inlier_cloud = pcd.select_by_index(ind)
    return inlier_cloud


def filter_out_radial_outlier(pcd, nb_points=16, radius=0.05):
    """
    
    """

    _, ind = pcd.remove_radius_outlier(nb_points=16, radius=0.05)
    inlier_cloud = pcd.select_by_index(ind)
    return inlier_cloud


def get_pcd_down(pcd, voxel_size):
    """
    
    """

    radius_normal = voxel_size * 2.0
    pcd_down = pcd.voxel_down_sample(voxel_size)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30)
    )
    return pcd_down


def get_pcd_fpfh(pcd_down, voxel_size):
    """
    
    """

    radius_feature = voxel_size * 1.5
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100)
    )
    return pcd_fpfh


def prepare_for_global_registration(scan, model, voxel_size):
    """
    
    """

    scan_down = get_pcd_down(scan, voxel_size)
    model_down = get_pcd_down(model, voxel_size)
    scan_fpfh = get_pcd_fpfh(scan_down, voxel_size)
    model_fpfh = get_pcd_fpfh(model_down, voxel_size)
    return scan_down, model_down, scan_fpfh, model_fpfh


def execute_global_registration(scan_down, model_down, scan_fpfh, model_fpfh, voxel_size):
    """
    
    """

    distance_threshold = 1.5 * voxel_size
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        scan_down, 
        model_down, 
        scan_fpfh, 
        model_fpfh, 
        True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        4, 
        [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.99),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
        ], 
        o3d.pipelines.registration.RANSACConvergenceCriteria(1000000, 0.999)
    )
    return result


def execute_fast_global_registration(scan_down, model_down, scan_fpfh, model_fpfh, voxel_size):
    """
    
    """

    distance_threshold = 10 * voxel_size
    result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        scan_down, 
        model_down,
        scan_fpfh,
        model_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold,
            iteration_number = 1000
            )
        )
    return result


def prepare_for_local_registration(scan, model, voxel_size):
    """
    
    """

    scan_down = get_pcd_down(scan, voxel_size)
    model_down = get_pcd_down(model, voxel_size)
    return scan_down, model_down


def execute_local_registration(scan_down, model_down):
    """

    """

    criteria = o3d.pipelines.registration.ICPConvergenceCriteria(1e-9, 1e-9, max_iteration=10000)
    result = o3d.pipelines.registration.registration_icp(
        scan_down,
        model_down,
        0.01,
        np.identity(4),
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        criteria
    )
    return result


def get_random_rotation_matrix():
    """
    
    """

    r = R.random()
    rotation_matrix_3x3 = r.as_matrix()
    rotation_matrix_4x4 = np.eye(4)
    rotation_matrix_4x4[:3, :3] = rotation_matrix_3x3
    return rotation_matrix_4x4


def count_nearest_neigbors(scan, model, threshold):
    """
    
    """

    target_kd_tree = o3d.geometry.KDTreeFlann(model)
    counter = 0
    for point in scan.points:
        point = np.asarray(point)
        [k, idx, sqd] = target_kd_tree.search_radius_vector_3d(point, threshold)
        if k > 0:
            counter += 1
    return counter
