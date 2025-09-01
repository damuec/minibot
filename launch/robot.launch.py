import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, Command

# Image Transport Republisher factory function
def image_transport_republisher(transport, camera_topics):
    base_topic = camera_topics.split('/')[-1]

    return Node(
        package='image_transport',
        executable='republish',
        name=f'image_transport_republish_{transport}_{base_topic}',
        arguments=['raw', transport],
        remappings=[
            ('in', f'/camera/{camera_topics}'),
            (f'out/{transport}', f'/camera/{camera_topics}/{transport}'),
        ],
    )

def generate_launch_description():
    package_name = 'nav2_odin'
    package_dir = get_package_share_directory(package_name) 
    use_sim_time = LaunchConfiguration('use_sim_time')
    lidar_serial_port = LaunchConfiguration('lidar_serial_port')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='If true, use simulated clock'
    )
    declare_lidar_serial_port = DeclareLaunchArgument(
        'lidar_serial_port',
        default_value='/dev/rplidar',
        description='Specifying usb port to connected lidar'
    )

    # Paths to files
    robot_description_xacro_file = os.path.join(
        package_dir,
        'description',
        'robot.urdf.xacro'
    )
    rviz_config_file_dir = os.path.join(
        package_dir, 
        'config', 
        'minibot_config.rviz'
    )
    twist_mux_params_file = os.path.join(
        package_dir, 
        'config', 
        'twist_mux.yaml'
    )
    slam_params_file = os.path.join(
        package_dir,
        'config',
        'slam_toolbox_params.yaml'
    )

    # robot_state_publisher setup
    robot_description_config = Command([
        'xacro ', 
        robot_description_xacro_file, 
        ' use_ros2_control:=false'
    ])

    params = {
        'robot_description': ParameterValue(robot_description_config, value_type=str), 
        'use_sim_time': use_sim_time
    }

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Image Transport Republishers Nodes
    camera = 'image_raw'
    depth_camera = 'depth/image_raw'
    image_transports = ['compressed', 'compressedDepth', 'theora', 'zstd']  
    node_image_republishers = [image_transport_republisher(transport, depth_camera) 
                              for transport in image_transports]

    # Ackermann steering node
    steering_node = Node(
        package='nav2_odin',
        executable='steering_node',
        name='steering_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/esp32',
            'baud_rate': 115200,
            'timeout': 0.1,
            'command_timeout': 0.5,
        }]
    )

    # Twist mux node (updated to send commands directly to cmd_vel)
    node_twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[
            twist_mux_params_file,
            {'use_stamped': True},
        ],
        remappings=[
            ('cmd_vel_out', 'cmd_vel'),
        ],
    )

    # Twist stamper node
    node_twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        name='twist_stamper',
        output='screen',
        remappings=[
            ('cmd_vel_in', 'cmd_vel_smoothed'),
            ('cmd_vel_out', 'nav_vel'),
        ],
    )

    # RPLIDAR launch include
    node_rplidar_drive = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('sllidar_ros2'),
                'launch',
                'sllidar_c1_launch.py'
            )
        ]), 
        launch_arguments={
            'serial_port': lidar_serial_port, 
            'frame_id': 'lidar_frame'
        }.items()
    )

    # SLAM Toolbox node (for creating map on-the-fly)
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': False,  # Set to false for real robot
            'odom_frame': 'odom',
            'map_frame': 'map', 
            'base_frame': 'base_footprint',
            'scan_topic': '/scan',
            'mode': 'mapping',  # Create new map on startup
            'resolution': 0.05,
            'max_laser_range': 10.0,
            'transform_timeout': 0.2,
            'map_update_interval': 5.0,
            'minimum_travel_distance': 0.5,
            'minimum_travel_heading': 0.5,
            'do_loop_closing': True,
            'loop_search_maximum_distance': 3.0
        }]
    )

    # Nav2 launch with autostart disabled
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'params_file': os.path.join(package_dir, 'config', 'nav2_params.yaml'),
            'use_sim_time': use_sim_time,
            'autostart': 'false'  # Set to false to use our own lifecycle manager
        }.items()
    )

    # Custom lifecycle manager with proper timeouts
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'controller_server',
                'smoother_server', 
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
                'collision_monitor',
                'docking_server'
            ],
            'bond_timeout': 10.0,
            'configure_timeout': 60.0,
            'activate_timeout': 60.0,
            'cleanup_timeout': 5.0,
            'deactivate_timeout': 5.0,
            'shutdown_timeout': 5.0
        }]
    )

     
    # Create launch description and add actions
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_lidar_serial_port)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(steering_node)  
    ld.add_action(node_twist_mux)
    ld.add_action(node_twist_stamper)
    ld.add_action(node_rplidar_drive)
    ld.add_action(slam_toolbox)  # Add SLAM toolbox
    ld.add_action(nav2_launch)  
    ld.add_action(lifecycle_manager)  # Add the custom lifecycle manager

    return ld