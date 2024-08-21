
from sensor_msgs.msg import PointCloud2

import numpy as np
import open3d as o3d
from os import path, makedirs

from modules.conversion import (
    read_pointcloud,
    unpack_pointcloud
)


def check_path(check_path):
    """
    Check if the save path exists and if it does not, create empty directories so that it does.

    Args:
        check_path (string): The absolute path of which's existence is to be checked.
    """
    if not path.exists(check_path):
        return 0
    else:
        return 1


def create_path(create_path):
    """
    Create empty directories for a given path.

    Args:
        create_path (string): The absolute path at which directories are to be created.
    """
    makedirs(create_path)


def save_pointcloud(pointcloud, file_path):
    """
    Save a ROS2 PointCloud2 message as a PCD file.

    Args:
        pointcloud (PointClouds2): The ROS2 PointCloud2 msg.
        file_path (string): The path at which the file_data is to be saved.
    """
    point_generator = read_pointcloud(pointcloud, skip_nans=True)
    point_list = list(point_generator)
    xyz, rgb = unpack_pointcloud(point_list)
    
    file_data = o3d.geometry.PointCloud()
    file_data.points = o3d.utility.Vector3dVector(xyz)
    file_data.colors = o3d.utility.Vector3dVector(rgb / 255.0)

    o3d.io.write_point_cloud(file_data, file_path)
    