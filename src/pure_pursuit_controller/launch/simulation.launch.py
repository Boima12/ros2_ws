import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'pure_pursuit_controller'
    pkg_share = get_package_share_directory(pkg_name)

    # 1. Path to SDF World
    # world_file = os.path.join(pkg_share, 'worlds', 'oval_track.sdf')
    world_file = os.path.join(pkg_share, 'worlds', 'line_track.sdf')
    
    # 2. Path to URDF
    urdf_file = os.path.join(pkg_share, 'urdf', 'ackermann_car.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 3. Launch Gazebo (gz_sim)
    # We use -r to run immediately
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # 4. Spawn the Robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'ackermann_car',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.2'
        ],
        output='screen'
    )

    # 5. Robot State Publisher (Publishes TF for the robot links)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
    )

    # 6. ROS-Gazebo Bridge
    # Bridges /cmd_vel (ROS->GZ) and /odom (GZ->ROS)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ],
        output='screen'
    )

    # 7. Your Pure Pursuit Node
    pure_pursuit = Node(
        package=pkg_name,
        executable='pure_pursuit_node',
        name='pure_pursuit_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        pure_pursuit
    ])
