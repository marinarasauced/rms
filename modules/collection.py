
from rms_msgs.msg import ViewPoint

import datetime
import open3d as o3d
from os import path, makedirs, listdir
import re

from modules.conversion import (
    read_pointcloud,
    unpack_pointcloud,
    convert_stl_to_pcd,
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

    o3d.io.write_point_cloud(file_path, file_data)


def get_config_path(rms_path):
    """
    
    """
    config_path = path.abspath(path.join(rms_path, "config"))
    return config_path


def get_scans_path(rms_path):
    """
    
    """
    date = datetime.datetime.now().strftime("%Y-%m-%d")
    time = datetime.datetime.now().strftime("%H:%M:%S")
    scan_path = path.abspath(path.join(rms_path, f"scans/{date}/"))
    if not check_path(scan_path):
        scans_path = path.join(scan_path, f"0001_{time}")
        create_path(scans_path)
        return scans_path
    else:
        pattern = re.compile(r"^(\d{4})_\d{2}:\d{2}:\d{2}$")
        numbers = []
    for item in listdir(scan_path):
        if path.isdir(path.join(scan_path, item)):
            match = pattern.match(item)
            if match:
                numbers.append(match.group(1))
    if numbers:
        index = int(max(numbers)) + 1
        scans_path = path.join(scan_path, f"{index:04d}_{time}")
        create_path(scans_path)
        return scans_path
    else:
        scans_path = path.join(scan_path, f"0001_{time}")
        create_path(scans_path)
        return scans_path


def get_model_path(rms_path, model_name):
    """
    
    """
    pcd_path = path.abspath(path.join(rms_path, f"models/pcd/{model_name}.pcd"))
    stl_path = path.abspath(path.join(rms_path, f"models/stl/{model_name}.stl"))

    if not check_path(pcd_path):
        if check_path(stl_path):
            convert_stl_to_pcd(stl_path, pcd_path)
            return pcd_path
        else:
            return None
    else:
        return pcd_path


def get_viewpoints(config_path, manipulator):
    """
    Load viewpoints from a file and convert them into an array of ViewPoint messages.

    Args:
        config_path (str): Path to the configuration directory.
        manipulator (str): The manipulator name to determine the file name.

    Returns:
        List[ViewPoint]: A list of ViewPoint messages.
    """
    file_path = path.abspath(path.join(config_path, f"{manipulator}_viewpoints.txt"))
    with open(file_path, "r") as file:
        lines = file.readlines()
    
    viewpoints = []
    for line in lines:
        values = [float(x) for x in line.strip().split(",")]
        viewpoint = ViewPoint()
        viewpoint.position.x = values[1]
        viewpoint.position.y = values[2]
        viewpoint.position.z = values[3]
        viewpoint.orientation.x = 0.0
        viewpoint.orientation.y = 0.0
        viewpoint.orientation.z = 0.0
        viewpoint.orientation.w = 0.0
        viewpoints.append(viewpoint)

    return viewpoints
