import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.getenv('TURTLEBOT3_MODEL', 'burger')  # avoid KeyError

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    # Use your package + map/mytestmap.yaml by default
    map_dir = LaunchConfiguration(
        'map',
        default=PathJoinSubstitution([
            FindPackageShare('Project_Pkg'), 'map', 'project.map.yaml'
        ])
    )

    # You can keep TB3’s params or point to your own nav2_params.yaml
    param_file_name = TURTLEBOT3_MODEL + '.yaml'
    params_file = LaunchConfiguration(
        'params_file',
        default=PathJoinSubstitution([
            FindPackageShare('turtlebot3_navigation2'), 'param', param_file_name
        ])
        # or: default=PathJoinSubstitution([FindPackageShare('Project_Pkg'), 'config', 'nav2_params.yaml'])
    )

    nav2_launch = PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'])
    rviz_config = PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'rviz', 'nav2_default_view.rviz'])

    return LaunchDescription([
        DeclareLaunchArgument('map',         default_value=map_dir,     description='Path to map YAML'),
        DeclareLaunchArgument('params_file', default_value=params_file, description='Nav2 params'),
        DeclareLaunchArgument('use_sim_time', default_value='True',     description='Use Gazebo clock'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': params_file
            }.items()
        ),

        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])
