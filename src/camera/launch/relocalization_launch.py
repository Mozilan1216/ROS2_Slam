import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_dir = get_package_share_directory('camera')

    # 1. 启动 D435i (使用你已调通 IMU 的那个 launch)
    d435i_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'd435i_launch.py'))
    )

    # 2. 深度图转雷达
    depth_to_scan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        parameters=[os.path.join(pkg_dir, 'config', 'depth_to_scan_params.yaml')],
        remappings=[('image', '/camera/camera/depth/image_rect_raw'),
                    ('camera_info', '/camera/camera/depth/camera_info')]
    )

    # 3. 加载已有的地图
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[{'yaml_filename': os.path.join(pkg_dir, 'map', 'rtabmap_20260112.yaml')}]
    )
    
    map_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        parameters=[{'autostart': True, 'node_names': ['map_server']}]
    )

    # 4. RTAB-Map 定位模式 (引用你的 localization yaml)
    rtabmap_node = Node(
        package='rtabmap_slam', executable='rtabmap',
        parameters=[os.path.join(pkg_dir, 'config', 'rtabmap_params_localization.yaml'),
                   {'localization': True}],
        remappings=[
            ("rgb/image", "/camera/camera/color/image_raw"),
            ("depth/image", "/camera/camera/depth/image_rect_raw"),
            ("rgb/camera_info", "/camera/camera/color/camera_info"),
            ("imu", "/camera/camera/imu")
        ]
    )

    return LaunchDescription([d435i_cmd, depth_to_scan_node, map_server, map_manager, rtabmap_node])