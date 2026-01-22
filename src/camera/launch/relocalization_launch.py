import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 路径获取
    pkg_dir = get_package_share_directory('camera')
    rtabmap_launch_dir = os.path.join(get_package_share_directory('rtabmap_launch'), 'launch')
    # 确保此处指向你建好的数据库
    rtabmap_db_file = os.path.join(pkg_dir, 'map', 'rtabmap_20260112.db')
    rviz_config_path = os.path.join(pkg_dir, 'rviz', 'relocation.rviz')

    # 2. 启动相机驱动 (直接复用你 slam_launch.py 中的参数)
    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'd435i_launch.py')),
        launch_arguments={
            'align_depth.enable': 'true',
            'enable_sync': 'true',
            'global_time_enabled': 'true', # 强制对齐系统时间
            'initial_reset': 'true'         # 启动时重置硬件
        }.items()
    )

    # 3. 官方 RTAB-Map 重定位模式 (模仿 slam_launch.py 但开启定位)
    rtabmap_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rtabmap_launch_dir, 'rtabmap.launch.py')),
        launch_arguments={
            'localization': 'true',          # 核心：开启定位模式
            'database_path': rtabmap_db_file,
            'rtabmap_args': '--Mem/IncrementalMemory false --Mem/InitWMWithAllNodes true',
            'rgb_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'imu_topic': '/camera/camera/imu',
            'wait_imu_to_init': 'false',
            'frame_id': 'camera_link',
            'approx_sync': 'true',
            'qos': '1',                      # 使用 Reliable 模式
            'rviz': 'false',                 # 我们手动启动自定义 RViz
            'viz': 'true'                    # 开启你需要的 rtabmap_viz 界面
        }.items()
    )

    # 4. 手动启动其他配套节点
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': os.path.join(pkg_dir, 'map', 'rtabmap_20260112.yaml')}]
    )
    
    map_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        parameters=[{'autostart': True, 'node_names': ['map_server']}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path]
    )

    # 5. 延迟启动以确保驱动先就绪
    delayed_rtabmap = TimerAction(period=3.0, actions=[rtabmap_node])

    return LaunchDescription([
        camera_node,
        map_server,
        map_manager,
        delayed_rtabmap,
        rviz_node
    ])