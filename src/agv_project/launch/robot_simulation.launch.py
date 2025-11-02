import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Path Definitions
    pkg_agv_project = get_package_share_directory('agv_project')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # File Paths 
    world_file_name = 'warehouse.sdf' 
    world_path = os.path.join(pkg_agv_project, 'worlds', world_file_name)

    # Declare Launch Arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # LaunchConfiguration to handle the 'use_sim_time' variable
    use_sim_time = LaunchConfiguration('use_sim_time')



    # Start Gazebo Server & Client
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_path, 'use_sim_time': use_sim_time}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    #Start Robot State Publisher
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    #Spawning the Robot (P-Bot)
    start_x_pose = '-4.0'
    start_y_pose = '-3.0'
    start_z_pose = '0.01'
    start_yaw = '0.01' 

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': start_x_pose,
            'y_pose': start_y_pose,
            'z_pose': start_z_pose,
            'yaw': start_yaw,
            'use_sim_time': use_sim_time 
        }.items()
    )


    ld = LaunchDescription()

    # Add all actions 
    ld.add_action(declare_use_sim_time_cmd) 
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)

    return ld

