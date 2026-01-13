# SLAM视觉建图
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 1. 路径获取
    rs_launch_dir = os.path.join(get_package_share_directory('realsense2_camera'), 'launch')
    rtabmap_launch_dir = os.path.join(get_package_share_directory('rtabmap_launch'), 'launch')
    config_path = os.path.join(get_package_share_directory('camera'), 'config', 'd435i_params.yaml')

    # 2. 相机驱动节点 (加上 align_depth 强制开启)
    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rs_launch_dir, 'rs_launch.py')),
        launch_arguments={
            'ros_params_file': config_path,
            'align_depth.enable': 'true',
            'enable_accel': 'true',
            'enable_gyro': 'true',
            'unite_imu_method': '2',                                                     
        }.items()
    )

    # 3. SLAM 节点 (RTAB-Map)
    slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rtabmap_launch_dir, 'rtabmap.launch.py')),
        launch_arguments={
            'rtabmap_args': '--delete_db_on_start --Vis/MinInliers 10',
            'rgb_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'frame_id': 'camera_link',
            
            # --- 新增/修改 IMU 相关参数 ---
            'imu_topic': '/camera/camera/imu',    # 告诉 RTAB-Map IMU 话题在哪里
            'wait_imu_to_init': 'false',   # 改为 false！防止因为 IMU 没对齐导致整个系统卡死
            'use_imu_for_bundle_adjustment': 'true', # 利用 IMU 优化地图
            # ---------------------------
            
            'approx_sync': 'true',
            'visual_odometry': 'true',
            'qos': '1',
            'rviz': 'true',
            'viz': 'false'
        }.items()
    )

    # 4. 延迟启动 SLAM (防止相机还没初始化好 SLAM 就开始报错)
    delayed_slam_node = TimerAction(
        period=3.0,
        actions=[slam_node]
    )

    return LaunchDescription([
        camera_node,
        delayed_slam_node
    ])