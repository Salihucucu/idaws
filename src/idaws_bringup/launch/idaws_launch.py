"""
IDAWS Ana Launch Dosyası
────────────────────────
Tüm node'ları sırasıyla ve doğru argümanlarla başlatır.
sllidar_ros2 paketi workspace'e clone edilmiş olmalıdır.

Kullanım:
  ros2 launch idaws_bringup idaws_launch.py
  ros2 launch idaws_bringup idaws_launch.py use_yolo:=true
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ─── Launch argümanları ───
    use_yolo_arg = DeclareLaunchArgument(
        'use_yolo', default_value='false',
        description='YOLO duba tespitini etkinleştir/devre dışı bırak'
    )
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyACM0',
        description='Pixhawk seri port yolu'
    )
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/ttyUSB1',
        description='LiDAR seri port yolu'
    )

    # ─── Parametre dosyası ───
    bringup_dir = get_package_share_directory('idaws_bringup')
    params_file = os.path.join(bringup_dir, 'config', 'params.yaml')

    # ─── 1. pymavlink_controller_node ───
    pymavlink_node = Node(
        package='idaws_nodes',
        executable='pymavlink_controller_node',
        name='pymavlink_controller_node',
        output='screen',
        parameters=[
            params_file,
            {'serial_port': LaunchConfiguration('serial_port')},
        ],
    )

    # ─── 2. sllidar_ros2 (Slamtec LiDAR) ───
    sllidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': LaunchConfiguration('lidar_port'),
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }],
    )

    # ─── 3. yolo_vision_node ───
    yolo_node = Node(
        package='idaws_nodes',
        executable='yolo_vision_node',
        name='yolo_vision_node',
        output='screen',
        parameters=[
            params_file,
            {'use_yolo': LaunchConfiguration('use_yolo')},
        ],
    )

    # ─── 4. collision_avoidance_node ───
    collision_node = Node(
        package='idaws_nodes',
        executable='collision_avoidance_node',
        name='collision_avoidance_node',
        output='screen',
        parameters=[params_file],
    )

    # ─── 5. mission_manager_node ───
    mission_node = Node(
        package='idaws_nodes',
        executable='mission_manager_node',
        name='mission_manager_node',
        output='screen',
        parameters=[params_file],
    )

    # ─── 6. mission_logger_node ───
    logger_node = Node(
        package='idaws_nodes',
        executable='mission_logger_node',
        name='mission_logger_node',
        output='screen',
        parameters=[params_file],
    )

    # ─── 7. lidar_logger_node ───
    lidar_logger_node = Node(
        package='idaws_nodes',
        executable='lidar_logger_node',
        name='lidar_logger_node',
        output='screen',
        parameters=[params_file],
    )

    # ─── 8. web_ui_node ───
    webui_node = Node(
        package='idaws_nodes',
        executable='web_ui_node',
        name='web_ui_node',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        use_yolo_arg,
        serial_port_arg,
        lidar_port_arg,

        LogInfo(msg='═══ IDAWS USV Sistemi Başlatılıyor ═══'),

        pymavlink_node,
        sllidar_node,
        yolo_node,
        collision_node,
        mission_node,
        logger_node,
        lidar_logger_node,
        webui_node,

        LogInfo(msg='═══ Tüm node\'lar başlatıldı ═══'),
    ])
