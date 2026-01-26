import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 路径定义
    pkg_dir = get_package_share_directory('camera')
    rs_launch_dir = os.path.join(get_package_share_directory('realsense2_camera'), 'launch')
    rtabmap_launch_dir = os.path.join(get_package_share_directory('rtabmap_launch'), 'launch')
    
    # 传感器参数与 SLAM 参数
    d435i_config = os.path.join(pkg_dir, 'config', 'd435i_params.yaml')
    rtabmap_config = os.path.join(pkg_dir, 'config', 'rtabmap_params.yaml')

    # 2. Realsense 驱动节点
    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rs_launch_dir, 'rs_launch.py')),
        launch_arguments={
            'ros_params_file': d435i_config,
            'align_depth.enable': 'true',
            'enable_accel': 'true',
            'enable_gyro': 'true',
            'unite_imu_method': '2', # 线性插值对齐
            'enable_sync': 'true'
        }.items()
    )

    # 3. RTAB-Map 建图节点
    # 使用 Include 官方 launch 的方式，但注入你的 params_file
    mapping_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rtabmap_launch_dir, 'rtabmap.launch.py')),
        launch_arguments={
            # 核心：加载建图专用参数文件
            'params_file': rtabmap_config,
            
            # 基础设置
            'rtabmap_args': '--delete_db_on_start', # 建图前清空旧数据库
            'namespace': 'camera',                  # 匹配 YAML 中的层级
            'namespace': 'camera',
            'rtabmap_node_name': 'rtabmap',      # 显式指定 SLAM 节点名
            'rtabmap_viz_node_name': 'rtabmap_viz_gui', # 显式指定可视化节点名，防止冲突
            'frame_id': 'camera_link',
            
            # 话题映射
            'rgb_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'imu_topic': '/camera/camera/imu',
            
            # 视觉里程计与同步
            'visual_odometry': 'true',
            'approx_sync': 'true',
            'wait_imu_to_init': 'false',
            'qos': '1',
            'rviz': 'true',
            'viz': 'false'
        }.items()
    )

    # 4. 延迟启动，确保 D435i 硬件先就绪
    delayed_mapping = TimerAction(
        period=3.0,
        actions=[mapping_node]
    )

    return LaunchDescription([
        camera_node,
        delayed_mapping
    ])