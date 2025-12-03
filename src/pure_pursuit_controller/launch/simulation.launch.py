import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Tìm đường dẫn đến package của bạn
    pkg_path = get_package_share_directory('pure_pursuit_controller')

    # 2. Định nghĩa đường dẫn file World và file URDF
    world_path = os.path.join(pkg_path, 'worlds', 'oval_track.sdf')
    urdf_path = os.path.join(pkg_path, 'urdf', 'ackermann_car.urdf')

    # 3. Lệnh mở Gazebo với file World đã tạo
    start_gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen'
    )

    # 4. Node nạp nội dung file URDF robot lên topic (để Gazebo đọc được)
    # Chúng ta dùng xacro hoặc đọc thẳng text, ở đây đọc thẳng text cho đơn giản
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # 5. Node "Spawn" - Thả con robot vào trong Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', robot_desc,
            '-name', 'ackermann_bot',
            '-x', '-0.5', '-y', '0', '-z', '0.5' # Thả xe ở độ cao 0.5m để không bị kẹt đất
        ],
        output='screen'
    )

    return LaunchDescription([
        start_gazebo,
        spawn_robot
    ])
