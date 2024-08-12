
import copy
import glob
import open3d as o3d
from os import path
import numpy as np


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

    search_path = path.join(read_path, f"{manipulator}_tfs.txt")
    search_file = []
    search_file.extend(glob.glob(search_path))


def filter_by_axis(pcd, threshold, axis):
    """
    
    """


    array = np.asarray(pcd.points)
    indices = np.where(np.abs(array[:, axis]) < threshold)
    filtered = pcd.select_by_index(indices[0])
    return filtered


def get_pcd_down(pcd, voxel_size):
    """
    
    """

    radius_normal = voxel_size * 2
    pcd_down = pcd.voxel_down_sample(voxel_size)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30)
    )
    return pcd_down


def get_pcd_fpfh(pcd_down, voxel_size):
    """
    
    """

    radius_feature = voxel_size * 1.0
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100)
    )
    return pcd_fpfh


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
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
        ], 
        o3d.pipelines.registration.RANSACConvergenceCriteria(1000000, 0.1)
    )
    return result

# def draw_registration_result(scan, model):
#     """
    
#     """

#     scan.paint_uniform_color([1, 0.706, 0])
#     model.paint_uniform_color([0, 0.651, 0.929])
#     o3d.visualization.draw_geometries(
#         [scan, model],
#         zoom=0.4559,
#         front=[0.6452, -0.3036, -0.7011],
#         lookat=[1.9892, 2.0208, 1.8945],
#         up=[-0.2779, -0.9482, 0.1556]
#     )