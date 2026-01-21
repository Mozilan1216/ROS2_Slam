import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_dir = get_package_share_directory('camera')
    
    # 路径定义
    map_yaml_file = os.path.join(pkg_dir, 'map', 'rtabmap_20260112.yaml')
    rtabmap_db_file = os.path.join(pkg_dir, 'map', 'rtabmap_20260112.db')
    rviz_config_path = os.path.join(pkg_dir, 'rviz', 'relocation.rviz')

    # 1. 启动 D435i
    d435i_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'd435i_launch.py'))
    )

    # 2. 深度图转雷达
    depth_to_scan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        parameters=[os.path.join(pkg_dir, 'config', 'depth_to_scan_params.yaml')],
        remappings=[('image', '/camera/camera/depth/image_rect_raw'),
                    ('camera_info', '/camera/camera/depth/camera_info')]
    )

    # 3. 地图服务器 (强制定义 QoS)
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{
            'yaml_filename': map_yaml_file,
            'topic_name': 'map',
            'frame_id': 'map'
        }]
    )
    
    map_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server']
        }]
    )

    # 4. RTAB-Map 定位模式 (解决类型报错的关键点)
    rtabmap_node = Node(
        package='rtabmap_slam', executable='rtabmap',
        name='rtabmap',
        parameters=[{
            'database_path': rtabmap_db_file,
            'localization': True,
            'Mem/IncrementalMemory': 'false',   # 显式使用字符串
            'Mem/InitWMWithAllNodes': 'true',    # 显式使用字符串
            'frame_id': 'camera_link',
            'subscribe_depth': True,
            'subscribe_scan': True,
            'approx_sync': True,
            'use_action_for_goal': True,
            'qos_image': 2,    # 适配 Best Effort
            'qos_scan': 2      # 适配 Best Effort
        }],
        remappings=[
            ("rgb/image", "/camera/camera/color/image_raw"),
            ("depth/image", "/camera/camera/depth/image_rect_raw"),
            ("rgb/camera_info", "/camera/camera/color/camera_info"),
            ("scan", "/scan"),
            ("imu", "/camera/camera/imu")
        ],
        output='screen'
    )

    # 5. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        d435i_cmd,
        depth_to_scan_node,
        map_server,
        map_manager,
        rtabmap_node,
        rviz_node
    ])