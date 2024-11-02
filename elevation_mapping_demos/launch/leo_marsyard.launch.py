# example.launch.py

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    leo_gz_worlds = FindPackageShare('leo_gz_worlds')
    elevation_mapping_demos = FindPackageShare('elevation_mapping_demos')
    ros_gz_sim = FindPackageShare('ros_gz_sim')
    ros_gz_bridge = FindPackageShare('ros_gz_bridge')

    robot_desc = ParameterValue(
        Command(
            [
                'xacro ',
                PathJoinSubstitution([elevation_mapping_demos, 'urdf', 'leo_rover_depthcam.urdf.xacro'])
            ]
        )
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"use_sim_time": True},
            {"robot_description": robot_desc},
        ],
    )
    # Spawn a robot inside a simulation
    leo_rover = Node(
        package="ros_gz_sim",
        executable="create",
        name="ros_gz_sim_create",
        output="both",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "leo_rover",
            "-z",
            "1.4",
            "-x",
            "-17.4",
            "-y",
            "7.4"
        ],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': os.path.join(get_package_share_directory('leo_gz_worlds'), 'worlds', 'marsyard2022.sdf') + ' -r',
        }.items()
    )

    topics_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="topics_bridge",
        parameters=[
            {
                "config_file": PathJoinSubstitution([elevation_mapping_demos, "config", "gz_bridge", "leo_ros_gz_bridge.yaml"]),
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ],
        parameters=[
            {
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )


    # voexel_filter_container = ComposableNodeContainer(
    #     name='pcl_node_container',
    #     namespace='',
    #     package='rclcpp_components',
    #     executable='component_container_mt',
    #     composable_node_descriptions = [
    #         ComposableNode(
    #             package='pcl_ros',
    #             plugin='pcl_ros::VodelGrid',
    #             name='voxel_grid_filter',
    #             parameters=[{
    #                 'leaf_size': 0.05,
    #                 'filter_field_name': 'z',
    #                 'filter_limit_min': 0.1,
    #                 'filter_limit_max': 7.0,
    #                 'filter_limit_negative': False

    #             }],
    #             remappings=[
    #                 ('input', '/oak/points'),
    #                 ('output', '/oak/points_filtered')
    #             ]
    #         )
    #     ]
    # )


    return LaunchDescription([
        gz_sim,
        topics_bridge,
        clock_bridge,
        robot_state_publisher,
        leo_rover
    ])