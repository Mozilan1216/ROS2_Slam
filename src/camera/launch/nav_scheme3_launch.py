import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 路径配置
    camera_pkg = get_package_share_directory('camera')

    #config
    rtabmap_config_path = os.path.join(camera_pkg, 'config', 'rtabmap_params_localization.yaml')
    nav2_params_file = os.path.join(camera_pkg, 'config', 'nav2_params_scheme3.yaml')
    rviz_config_path = os.path.join(camera_pkg, 'rviz', 'navi.rviz') 

    map_yaml_file = os.path.join(camera_pkg, 'map', 'map_20260309.yaml') 
    rtabmap_db_file = '/home/mozilan/nav2_ws/src/camera/rtabmap/rtabmap_20260309.db'
    # map_yaml_file = os.path.join(camera_pkg, 'map', 'rtabmap_20260112.yaml') 
    # rtabmap_db_file = '/home/mozilan/nav2_ws/src/camera/rtabmap/rtabmap_220260112.db'
    
    # launch
    rs_launch_dir = os.path.join(get_package_share_directory('realsense2_camera'), 'launch')
    rtabmap_launch_dir = os.path.join(get_package_share_directory('rtabmap_launch'), 'launch')
    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

   
    # 1. 启动 Realsense 相机 (开启 IMU)
    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rs_launch_dir, 'rs_launch.py')),
        launch_arguments={
            'align_depth.enable': 'true',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '2',
            'enable_sync': 'true',
            'initial_reset': 'true', # 强制重置相机，解决没话题的问题
            # 降低一点分辨率以减轻计算压力，保证里程计稳定
            'depth_module.profile': '640x480x15',
            'rgb_camera.profile': '640x480x15'
        }.items()
    )

    # 2. 静态 TF (base_link -> camera_link)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link']
    )

    # 3. 深度图转激光雷达节点 (Scheme 3 的核心)
    depth_to_scan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        parameters=[{
            'scan_time': 0.033,
            'range_min': 0.2,
            'range_max': 8.0,
            'scan_height': 1,        # 扫描 1 像素的高度
            'output_frame': 'camera_link' # 伪雷达数据绑定在相机上
        }],
        remappings=[
            ('depth', '/camera/camera/aligned_depth_to_color/image_raw'),
            ('depth_camera_info', '/camera/camera/aligned_depth_to_color/camera_info'),
            ('scan', '/scan') # 输出话题
        ]
    )

    # 4. 静态地图发布
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': map_yaml_file, 'use_sim_time': False}]
    )
    map_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        parameters=[{'autostart': True, 'node_names': ['map_server']}]
    )

    # 5. RTAB-Map 定位模式 (提供 odom->base_link 和 map->odom)
    rtabmap_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rtabmap_launch_dir, 'rtabmap.launch.py')),
        launch_arguments={
            'localization': 'true',
            'database_path': rtabmap_db_file,
            'rtabmap_params_proxy': rtabmap_config_path,
            'frame_id': 'base_link',
            'visual_odometry': 'true',
            'publish_tf_odom': 'true',
            'approx_sync': 'true',
            'wait_imu_to_init': 'false', # 防止无限等待
            'rgb_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'imu_topic': '/camera/camera/imu',
            'qos': '1',
            'map_topic': '/rtabmap/map_ignore',
        }.items()
    )

    rviz_node = Node(
        package='rviz2', 
        executable='rviz2', 
        name='rviz2',
        arguments=['-d', rviz_config_path], 
        output='screen'
    )

    # 6. Nav2 纯导航
    nav2_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params_file,
            'autostart': 'true',
        }.items()
    )

    return LaunchDescription([
        # SetEnvironmentVariable('MESA_GL_VERSION_OVERRIDE', '3.3'),
        # SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        camera_node,
        static_tf,
        depth_to_scan_node, # 启动伪雷达转换
        map_server_node,
        map_lifecycle_node,
        rviz_node,
        TimerAction(period=3.0, actions=[rtabmap_node]),
        # TimerAction(period=8.0, actions=[nav2_node])
    ])