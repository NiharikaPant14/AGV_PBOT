import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # Path to package
    pkg_agv_project = get_package_share_directory('agv_project')

    # Path to slam_toolbox package
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    # Launch the Simulation

    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_agv_project, 'launch', 'robot_simulation.launch.py')
        )
    )

    # Launch SLAM Toolbox 
    # Path to the slam_params.yaml file
    slam_params_file = os.path.join(pkg_agv_project, 'config', 'slam_params.yaml')

    # SLAM Toolbox node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file] 
    )

    # Launch RViz 
    rviz_config_file = os.path.join(
        pkg_slam_toolbox, 'config', 'slam_toolbox_default.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file], 
        parameters=[{'use_sim_time': True}]
    )

    # Final Launch Description
    return LaunchDescription([
        simulation_launch,
        slam_toolbox_node,
        rviz_node
    ])