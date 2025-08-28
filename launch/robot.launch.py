import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, Command

# Image Transport Republishers
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
        default_value='/dev/ttyUSB0',
        description='Specifying usb port to connected lidar'
    )
    
    # Declare the path to files
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

    # robot_state_publisher setup    
    # Note: We're setting use_ros2_control to false since we're not using it
    robot_description_config = Command ([
        'xacro ', 
        robot_description_xacro_file, 
        ' use_ros2_control:=false'
        ])
    
    params = {
        'robot_description': ParameterValue(robot_description_config, value_type=str), 
        'use_sim_time': use_sim_time
    }

    # robot_state_publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
 
    # Image Transport Republishers Node
    camera = 'image_raw'
    depth_camera = 'depth/image_raw'
    image_transports = ['compressed','compressedDepth', 'theora', 'zstd']  
    node_image_republishers = [image_transport_republisher(transport, depth_camera) 
                          for transport in image_transports]

    # Your Ackermann steering node
    steering_node = Node(
        package='nav2_odin',
        executable='ackermann_driver',
        name='ackermann_driver',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB1',
            'baudrate': 115200,
            'wheelbase': 0.325,
            'max_steer_angle': 0.4189,
            'max_throttle': 200,
            'max_linear_vel': 0.5,
        }]
    )

    # Update twist_mux to send commands directly to cmd_vel (not to diff_drive_controller)
    node_twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name='twist_mux',
        output='screen',
        parameters=[
            twist_mux_params_file,
            {'use_stamped': True},
        ],
        remappings=[
            ('cmd_vel_out', 'cmd_vel'),  # Changed from 'diff_drive_controller/cmd_vel'
        ],
    )

    # Update twist_stamper to work with your setup
    node_twist_stamper = Node(
        package="twist_stamper",
        executable="twist_stamper",
        name='twist_stamper',
        output='screen',
        remappings=[
            ('cmd_vel_in', 'cmd_vel_smoothed'),
            ('cmd_vel_out', 'nav_vel'),
        ],
    )

    node_rplidar_drive = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('sllidar_ros2'),
                    'launch',
                    'sllidar_c1_launch.py'
                )]), 
                launch_arguments={
                    'serial_port': lidar_serial_port, 
                    'frame_id': 'lidar_frame'
                    }.items()
    )
    
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
        'autostart': 'true'
    }.items()
)
    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_lidar_serial_port)

    ld.add_action(node_robot_state_publisher)
    ld.add_action(steering_node)  
    ld.add_action(node_twist_mux)
    ld.add_action(node_twist_stamper)
    for node_republisher in node_image_republishers:
        ld.add_action(node_republisher)
    ld.add_action(node_rplidar_drive)
    ld.add_action(nav2_launch)  

    return ld