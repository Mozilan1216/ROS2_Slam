import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # ================= 1. 路径配置 =================
    camera_pkg = get_package_share_directory('camera')
    # 确保地图文件名与你提供的一致 
    map_yaml_file = os.path.join(camera_pkg, 'map', 'rtabmap_20260112.yaml') 
    loc_params_file = os.path.join(camera_pkg, 'config', 'rtabmap_params_localization.yaml')
    nav2_params_file = os.path.join(camera_pkg, 'config', 'nav2_params.yaml')

    # 获取官方功能包的 launch 路径 
    rs_launch_dir = os.path.join(get_package_share_directory('realsense2_camera'), 'launch')
    rtabmap_launch_dir = os.path.join(get_package_share_directory('rtabmap_launch'), 'launch')
    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    # ================= 2. 节点配置 =================

    # A. Realsense 驱动 (设置较低帧率减小压力)
    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rs_launch_dir, 'rs_launch.py')),
        launch_arguments={
            'align_depth.enable': 'true',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '2',
            'initial_reset': 'true',           # 强制重置硬件 
            'depth_module.profile': '640x480x15',
            'rgb_camera.profile': '640x480x15'
        }.items()
    )

    # B. RTAB-Map 定位模式 (关闭原生 Viz 窗口防止报错) 
    rtabmap_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rtabmap_launch_dir, 'rtabmap.launch.py')),
        launch_arguments={
            'rtabmap_params_proxy': loc_params_file,
            'visual_odometry': 'true',
            'frame_id': 'camera_link',
            'approx_sync': 'true',
            'qos': '1',
            'viz': 'false'  # 关键：设为 false 避免 OpenGL 报错 
        }.items()
    )

    # C. Nav2 导航堆栈
    nav2_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': 'false',
            'params_file': nav2_params_file,
            'autostart': 'true'
        }.items()
    )

    # ================= 3. 组合与执行 =================
    return LaunchDescription([
        # 强制本地通信，解决 WiFi 干扰 
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        
        camera_node,
        # 延迟 3 秒启动定位和导航，等待相机初始化完成 
        TimerAction(period=3.0, actions=[rtabmap_node, nav2_node])
    ])