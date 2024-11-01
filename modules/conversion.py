
from sensor_msgs.msg import PointCloud2, PointField

import ctypes
import math
import numpy as np
import open3d as o3d
from os import path
import struct
import sys

_DATATYPES = {PointField.INT8: ('b', 1), PointField.UINT8: ('B', 1), PointField.INT16: ('h', 2),
              PointField.UINT16: ('H', 2), PointField.INT32: ('i', 4), PointField.UINT32: ('I', 4),
              PointField.FLOAT32: ('f', 4), PointField.FLOAT64: ('d', 8)}


def read_pointcloud(cloud, field_names=None, skip_nans=False, uvs=[]):
    assert isinstance(cloud, PointCloud2), 'cloud is not a sensor_msgs.msg.PointCloud2'
    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
    width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, math.isnan
    unpack_from = struct.Struct(fmt).unpack_from

    if skip_nans:
        if uvs:
            for u, v in uvs:
                p = unpack_from(data, (row_step * v) + (point_step * u))
                has_nan = False
                for pv in p:
                    if isnan(pv):
                        has_nan = True
                        break
                if not has_nan:
                    yield p
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    p = unpack_from(data, offset)
                    has_nan = False
                    for pv in p:
                        if isnan(pv):
                            has_nan = True
                            break
                    if not has_nan:
                        yield p
                    offset += point_step
    else:
        if uvs:
            for u, v in uvs:
                yield unpack_from(data, (row_step * v) + (point_step * u))
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    yield unpack_from(data, offset)
                    offset += point_step


def _get_struct_fmt(is_bigendian, fields, field_names=None):
    fmt = '>' if is_bigendian else '<'

    offset = 0
    for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print('Skipping unknown PointField datatype [%d]' % field.datatype, file=sys.stderr)
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt


def unpack_rgb(packed):
    s = struct.pack('>f', packed)
    i = struct.unpack('>l', s)[0]
    pack = ctypes.c_uint32(i).value
    r = (pack & 0x00FF0000) >> 16
    g = (pack & 0x0000FF00) >> 8
    b = (pack & 0x000000FF)

    return [r, g, b]


def unpack_pointcloud(point_list):
    """
    Unpack xyz coordinates and rgb colors from a list of pointcloud data.

    Args:
        point_list (list): A list of points where each point is a tuple containing
                           xyz coordinates and possibly rgb data.

    Returns:
        tuple:
            - xyz (np.ndarray): A NumPy array of shape (N, 3) containing the x, y, z coordinates.
            - rgb (np.ndarray): A NumPy array of shape (N, 3) containing the r, g, b color values.
    """
    xyz = np.array([[point[0], point[1], point[2]] for point in point_list])
    rgb = np.array([unpack_rgb(point[3]) for point in point_list]) if len(point_list[0]) > 3 else np.zeros((len(point_list), 3))
    return xyz, rgb


def convert_stl_to_pcd(stl_file, pcd_file):
    """
    Convert an STL file to a PCD file.

    Args:
        stl_file (str): Path to the input STL file.
        pcd_file (str): Path to the output PCD file.
    """
    stl = o3d.io.read_triangle_mesh(stl_file)
    pcd = stl.sample_points_uniformly(number_of_points=100000)
    o3d.io.write_point_cloud(pcd_file, pcd)
