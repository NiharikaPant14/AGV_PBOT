import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Path Definitions
    pkg_agv_project = get_package_share_directory('agv_project')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # File Paths 
    map_file_path = os.path.join(pkg_agv_project, 'maps', 'warehouse.yaml')
    nav2_params_file = os.path.join(pkg_agv_project, 'config', 'nav2_params.yaml')
    nav2_rviz_config_file = os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')
    
    # Launch Arguments 
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # LaunchConfiguration for all variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Simulation launch (Robot Spawner) 
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_agv_project, 'launch', 'robot_simulation.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Map Server launch
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_file_path,
            'use_sim_time': use_sim_time,
            'topic_name': 'map',
            'frame_id': 'map'
        }]
    )
    
    # Static TF Publisher (map -> odom)

    static_tf_map_odom = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='static_tf_map_odom',
    output='screen',
    arguments=[
        '--x', '0', '--y', '0', '--z', '0',
        '--roll', '0', '--pitch', '0', '--yaw', '0',
        '--frame-id', 'map',
        '--child-frame-id', 'odom'
    ],
    parameters=[{'use_sim_time': use_sim_time}]
    )

    
    #Lifecycle Manager for Map Server 
    map_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'autostart': True,
            'use_sim_time': use_sim_time,
            'node_names': ['map_server']
        }]
    )
    
    # Launch AMCL Node
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
    )
    
    #Navigation Stack (BT Navigator)
    bt_navigator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map_subscribe_transient_local': 'true',
            'params_file': nav2_params_file,
            'autostart': 'True',
            'use_sim_time': use_sim_time,
            'map': map_file_path,
            'use_map_server': 'False',
            'use_amcl': 'False'
        }.items()
    )
    
    # Launch RViz 
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', nav2_rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}] 
    )
    
    return LaunchDescription([
        declare_use_sim_time_cmd,
        simulation_launch,
        map_server_node,
        static_tf_map_odom,
        map_lifecycle_manager,
        amcl_node,
        bt_navigator_launch,
        rviz_node
    ])
