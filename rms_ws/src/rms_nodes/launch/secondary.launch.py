from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """

    """
    # Declare the launch argument for the robot model
    robot_model = LaunchConfiguration("robot_model")
    robot_model_ = DeclareLaunchArgument(
        "robot_model",
        default_value="vx300s",
        description="name of the manipulator"
    )

    # Include the manipulator launch file
    manipulator_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("rms_nodes"), "/launch/manipulator.launch.py"]
        ),
        launch_arguments={
            "robot_model": robot_model
        }.items()
    )

    # Node to collect pointclouds at viewpoints
    collection_server_node = Node(
        package='rms_nodes',
        executable='collect_pointclouds_at_viewpoints',
        name='collect_at_viewpoints_server',
        namespace=robot_model,
        parameters=[{
            'robot_model': robot_model,
        }],
    )

    grip_object_node = Node(
        package='rms_nodes',
        executable='grip_object_to_scan',
        name='grip_object_to_scan',
        namespace=robot_model,
        parameters=[{
            'robot_model': robot_model,
        }],
    )

    # Create the launch description and add the actions
    ld = LaunchDescription()
    ld.add_action(robot_model_)
    ld.add_action(manipulator_launch_file)
    ld.add_action(collection_server_node)
    ld.add_action(grip_object_node)

    return ld
