from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rms_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
                (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
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
            'collect_at_viewpoints_server = rms_nodes.collect_at_viewpoints_server:main',
            'register_to_model_server = rms_nodes.register_to_model_server:main',
            'config1 = rms_nodes._configuration1:main',
            'config2 = rms_nodes._configuration2:main',
        ],
    },
)
