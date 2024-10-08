import os

from sensor_msgs.msg import PointCloud2, PointField

import copy
import ctypes
import datetime
import glob
import open3d as o3d
import math
import numpy as np
import struct
import sys

_DATATYPES = {PointField.INT8: ('b', 1), PointField.UINT8: ('B', 1), PointField.INT16: ('h', 2),
              PointField.UINT16: ('H', 2), PointField.INT32: ('i', 4), PointField.UINT32: ('I', 4),
              PointField.FLOAT32: ('f', 4), PointField.FLOAT64: ('d', 8)}


def get_viewpoints(read_path):
    """
    
    """
    with open(read_path, "r") as file:
        lines = file.readlines()
    data = []
    for line in lines:
        values = [float(x) for x in line.strip().split(",")]
        data.append(values)
    return np.asarray(data)


def get_pointcloud(data, status, robot_model, counter, write_path):
    if data and status:
        scan_name = f"{robot_model}_{counter:03d}.pcd"
        scan_path = os.path.join(write_path, scan_name)
        write_pointcloud(data, scan_path)


def write_pointcloud(data, path):
    """
    
    """
    gen = read_pcd(data, skip_nans=True)
    ints = list(gen)
    xyz = np.array([[x[0], x[1], x[2]] for x in ints])
    rgb = np.array([unpack_rgb(x[3]) for x in ints])
    o3d_cloud = o3d.geometry.PointCloud()
    o3d_cloud.points = o3d.utility.Vector3dVector(xyz)
    o3d_cloud.colors = o3d.utility.Vector3dVector(rgb / 255.0)
    o3d.io.write_point_cloud(path, o3d_cloud)


def read_pcd(cloud, field_names=None, skip_nans=False, uvs=[]):
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


#
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


#
def unpack_rgb(packed):
    s = struct.pack('>f', packed)
    i = struct.unpack('>l', s)[0]
    pack = ctypes.c_uint32(i).value
    r = (pack & 0x00FF0000) >> 16
    g = (pack & 0x0000FF00) >> 8
    b = (pack & 0x000000FF)

    return [r, g, b]


def write_pcd_from_stl(read, write):
    """
    Write a PCD file equivalent to an STL file.

    @param read: The path to the STL file.
    @param write: The path to the PCD file.
    """

    if not os.path.exists(write):
        os.makedirs(write)
    stl = o3d.io.read_triangle_mesh(read)
    pcd = stl.sample_points_uniformly(1000000)
    o3d.io.write_point_cloud(write, pcd)

