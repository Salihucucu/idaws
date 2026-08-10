"""
IDAWS Simulation Launch (ROS 2 nodes + gz bridge)
--------------------------------------------------
Run AFTER Gazebo and SITL are already running (via start_sim.sh).
Bridges Gazebo LiDAR to /scan, Gazebo camera to /camera/image_raw,
and starts all IDAWS nodes.

Usage:
  ros2 launch idaws_bringup sim_launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    bringup_dir = get_package_share_directory('idaws_bringup')
    params_file = os.path.join(bringup_dir, 'sim', 'config', 'sim_params.yaml')

    # Kayıt dizini dışarıdan yönlendirilebilir. Docker'da bu, host'a bağlanan
    # birime (recordings/) işaret eder; ayarlanmazsa YAML'daki yol geçerlidir.
    # Şartname teslimleri (mp4/csv) bu dizine yazıldığı için konteyner içinde
    # kalan bir yola düşmesi teslimlerin kaybolması demek.
    log_dir = os.environ.get('IDAWS_LOG_DIR', '')
    log_override = [{'log_dir': log_dir}] if log_dir else []
    rec_override = [{'record_dir': log_dir}] if log_dir else []

    # ros_gz_bridge: Gazebo LiDAR → /scan
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        output='screen',
        arguments=[
            '/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        remappings=[
            ('/lidar', '/scan'),
        ],
    )

    # ros_gz_image: Gazebo kamerası → /camera/image_raw
    # Ezel model.sdf içindeki camera_sensor doğrudan /camera/image_raw adıyla
    # yayın yapar, böylece image_transport remap'ine gerek kalmaz.
    camera_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='camera_bridge',
        output='screen',
        arguments=['/camera/image_raw'],
    )

    # IDAWS nodes with sim parameters
    pymavlink_node = Node(
        package='idaws_nodes',
        executable='pymavlink_controller_node',
        name='pymavlink_controller_node',
        output='screen',
        parameters=[params_file],
    )

    yolo_node = Node(
        package='idaws_nodes',
        executable='yolo_vision_node',
        name='yolo_vision_node',
        output='screen',
        parameters=[params_file] + rec_override,
    )

    collision_node = Node(
        package='idaws_nodes',
        executable='collision_avoidance_node',
        name='collision_avoidance_node',
        output='screen',
        parameters=[params_file],
    )

    mission_node = Node(
        package='idaws_nodes',
        executable='mission_manager_node',
        name='mission_manager_node',
        output='screen',
        parameters=[params_file],
    )

    logger_node = Node(
        package='idaws_nodes',
        executable='mission_logger_node',
        name='mission_logger_node',
        output='screen',
        parameters=[params_file] + log_override,
    )

    lidar_logger_node = Node(
        package='idaws_nodes',
        executable='lidar_logger_node',
        name='lidar_logger_node',
        output='screen',
        parameters=[params_file] + log_override,
    )

    webui_node = Node(
        package='idaws_nodes',
        executable='web_ui_node',
        name='web_ui_node',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        LogInfo(msg='=== IDAWS Sim: Bridge + Nodes ==='),

        lidar_bridge,
        camera_bridge,
        pymavlink_node,
        yolo_node,
        collision_node,
        mission_node,
        logger_node,
        lidar_logger_node,
        webui_node,

        LogInfo(msg='=== All IDAWS nodes launched ==='),
    ])
