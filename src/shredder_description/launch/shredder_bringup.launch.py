from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    shredder_dir = get_package_share_directory('shredder_description')   
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    # tb3_dir = get_package_share_directory('turtlebot3_gazebo')
    # Declare arguments
    declared_arguments = []
    
    declared_arguments.append(DeclareLaunchArgument(
        'x_pose', default_value='5.0',
        description='X position to spawn the robot'))
    
    declared_arguments.append(DeclareLaunchArgument(
        'y_pose', default_value='1.0', 
        description='Y position to spawn the robot'))
        
    # declared_arguments.append(DeclareLaunchArgument(
    #     'world', default_value=os.path.join(shredder_dir, 'worlds', 'turtlebot3_house.world'),
    #     description='Full path to world model file to load'))
    
    declared_arguments.append(DeclareLaunchArgument(
        'world', default_value=os.path.join(shredder_dir, 'worlds', 'simple_baylands.sdf'),
        description='Full path to world model file to load'))
    
    # LaunchConfiguration variables
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    world = LaunchConfiguration('world')
    frame_prefix = LaunchConfiguration('frame_prefix', default='')
    
    # Load URDF
    urdf_file_name = "shredder_core.xacro"
    urdf_path = os.path.join(shredder_dir, 'urdf', urdf_file_name)
    
    # with open(urdf_path, 'r') as infp:
    #     robot_desc = infp.read()
   
    # Set environment for Gazebo resources
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(shredder_dir, 'models'))
    
    # Launch Gazebo server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v4 ', world]}.items()
    )
    
    # Launch Gazebo client
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v4'}.items()
    )
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            'use_sim_time': False,
            'robot_description': Command(['xacro ', urdf_path]),
            'frame_prefix': PythonExpression(["'", frame_prefix, "/'"])
        }],
    )
    
    # Spawn robot directly (SINGLE SPAWN POINT)
    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_shredder',
        arguments=[
            '-name', 'shredder',
            '-topic', "/robot_description",
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.2'  # Slightly higher to avoid ground collision
        ],
        output='screen',
    )
    
    # ROS-Gazebo Bridge
    bridge_params = os.path.join(shredder_dir, 'params', 'shredder_bridge.yaml')
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p', f'config_file:={bridge_params}',
        ],
        output='screen',
    )
    
    # RViz
    rviz_config_file = os.path.join(shredder_dir, "rviz", "view.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )
    
    return LaunchDescription(
        declared_arguments + [
            gzserver_cmd,
            gzclient_cmd,
            robot_state_publisher_node,
            spawn_robot_node,  # ONLY ONE SPAWN CALL
            bridge_node,
            set_env_vars_resources,
            rviz_node,
        ]
    )