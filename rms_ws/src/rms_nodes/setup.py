from glob import glob
from os import path
from setuptools import find_packages, setup

package_name = 'rms_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            path.join("share", package_name, "launch"),
            glob(path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marina Nelson',
    maintainer_email='marinarasauced@outlook.com',
    description='ROS2 nodes for RMS PCD collection and registration.',
    license='Private License',
    entry_points={
        'console_scripts': [
            'save_pointclouds = src.save_pointclouds:main',
            'collect_pointclouds_at_viewpoints = src.collect_pointclouds_at_viewpoints:main',
            'register_pointclouds_to_model = src.register_pointclouds_to_model:main',
            'main = src.main:main',
            'grip = src.grip_demo:main',
            'grip_object_to_scan = src.grip_object_to_scan:main',
            'demo1 = src.demo1:main',
            'demo2 = src.demo2:main',
            'demo3 = src.demo3:main',
        ],
    },
)
