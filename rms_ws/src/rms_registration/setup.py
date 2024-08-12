from setuptools import find_packages, setup

package_name = 'rms_registration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='viperx250',
    maintainer_email='FirstNameLastName@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'global_registration_server = src.global_registration_server:main',
            'local_registration_server = src.local_registration_server:main',
        ],
    },
)
