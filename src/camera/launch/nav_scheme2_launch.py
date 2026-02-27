import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    camera_pkg = get_package_share_directory('camera')
    
    rtabmap_db_file = '/home/mozilan/nav2_ws/src/camera/rtabmap/rtabmap_20260112.db'
    # 引入你之前建好的静态地图 yaml 文件
    map_yaml_file = os.path.join(camera_pkg, 'map', 'rtabmap_20260112.yaml')
    
    nav2_params_file = os.path.join(camera_pkg, 'config', 'nav2_params_scheme2.yaml')
    rtabmap_config_path = os.path.join(camera_pkg, 'config', 'rtabmap_params_localization.yaml')
    rviz_config_path = os.path.join(camera_pkg, 'rviz', 'navi.rviz') 

    # 1. 启动 Realsense 相机 (完整恢复 IMU 与分辨率参数)
    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')),
        launch_arguments={
            'align_depth.enable': 'true',
            'enable_gyro': 'true',          # 必须恢复：激活 IMU
            'enable_accel': 'true',         # 必须恢复：激活加速度计
            'unite_imu_method': '2',        # 必须恢复：同步 IMU 时间戳
            'enable_sync': 'true',
            'depth_module.profile': '640x480x15', # 恢复原始配置以稳定视觉里程计
            'rgb_camera.profile': '640x480x15'
        }.items()
    )

    # 2. 静态 TF (补全 base_link)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link']
    )

    # 3. 静态地图发布服务器 (秒发地图，防止 Nav2 卡死)
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

    # 4. RTAB-Map 纯定位模式
    rtabmap_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py')),
        launch_arguments={
            'localization': 'true',
            'database_path': rtabmap_db_file,
            'rtabmap_params_proxy': rtabmap_config_path,
            'frame_id': 'base_link',
            'visual_odometry': 'true',
            'publish_tf_odom': 'true',
            'approx_sync': 'true',
            'rgb_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'imu_topic': '/camera/camera/imu',
            'qos': '1',
            # 关键：将 RTAB-Map 生成的动态地图扔到闲置话题，避免与 map_server 冲突
            'map_topic': '/rtabmap/map_ignore',
        }.items()
    )

    # 5. Nav2 纯导航堆栈 (去除 AMCL)
    nav2_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params_file,
            'autostart': 'true',
        }.items()
    )

    rviz_node = Node(
        package='rviz2', executable='rviz2', arguments=['-d', rviz_config_path], output='screen'
    )

    return LaunchDescription([
        SetEnvironmentVariable('MESA_GL_VERSION_OVERRIDE', '3.3'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        camera_node,
        static_tf,
        map_server_node,
        map_lifecycle_node,
        rviz_node,
        TimerAction(period=3.0, actions=[rtabmap_node]),
        # Nav2 会立刻拿到 map_server 发来的地图，顺利完成节点激活
        TimerAction(period=6.0, actions=[nav2_node])
    ])