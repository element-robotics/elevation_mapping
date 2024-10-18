# example.launch.py

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer


def generate_launch_description():

    lunarsim_gz_bringup_dir = FindPackageShare('lunarsim_gz_bringup')

    marsyard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([lunarsim_gz_bringup_dir, 'launch', 'lunarsim_world.launch.py'])
        ),
        launch_arguments={
            'world' : 'marsyard2022.sdf',
            'z_pose' : '5.0'
        }.items()
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
        marsyard_launch,
        pose_with_cov_relay,
    ])