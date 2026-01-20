import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 路径配置
    camera_pkg = get_package_share_directory('camera')
    map_yaml_file = os.path.join(camera_pkg, 'map', 'rtabmap_20260112.yaml') 
    loc_params_file = os.path.join(camera_pkg, 'config', 'rtabmap_params_localization.yaml')
    nav2_params_file = os.path.join(camera_pkg, 'config', 'nav2_params.yaml')
    rviz_config_path = os.path.join(camera_pkg, 'rviz', 'D435i.rviz')

    rs_launch_dir = os.path.join(get_package_share_directory('realsense2_camera'), 'launch')
    rtabmap_launch_dir = os.path.join(get_package_share_directory('rtabmap_launch'), 'launch')
    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    # A. Realsense 驱动 (开启 IMU 和 深度对齐)
    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rs_launch_dir, 'rs_launch.py')),
        launch_arguments={
            'align_depth.enable': 'true',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '2',
            'depth_module.profile': '640x480x15',
            'rgb_camera.profile': '640x480x15'
        }.items()
    )

    # B. RTAB-Map 定位模式
    rtabmap_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rtabmap_launch_dir, 'rtabmap.launch.py')),
        launch_arguments={
            'rtabmap_params_proxy': loc_params_file,
            'database_path': '/home/mozilan/nav2_ws/src/camera/rtabmap/rtabmap_20260112.db',
            'localization': 'true',  # 激活定位模式，不再添加新节点到数据库
            'visual_odometry': 'true',
            'frame_id': 'camera_link',  # 无底盘时，相机就是机器人中心
            'approx_sync': 'true',
            'qos': '1',
            'viz': 'false',
            'use_sim_time': 'false',
            # 显式映射话题，确保与 RealSense 发布的一致
            'rgb_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'imu_topic': '/camera/camera/imu',
            'wait_imu_to_init': 'false',
        }.items()
    )

    # 添加 RViz 节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # C. Nav2 导航堆栈
    nav2_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml_file,# 这里会启动 map_server 负载该 yaml 文件
            'use_sim_time': 'false',
            'params_file': nav2_params_file,
            'autostart': 'true'
        }.items()
    )

    return LaunchDescription([
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        camera_node,
        rviz_node, # 立即启动 RViz
        TimerAction(period=3.0, actions=[rtabmap_node, nav2_node])
    ])