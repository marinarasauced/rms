from setuptools import find_packages, setup

package_name = 'rms_registration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marina Nelson',
    maintainer_email='marinarasauced@outlook.com',
    description='ROS2 nodes for RMS PCD registration.',
    license='Private License',
    entry_points={
        'console_scripts': [
            'register_to_model_server = src.register_to_model:main',
        ],
    },
)
