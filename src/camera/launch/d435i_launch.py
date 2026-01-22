import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 1. 获取路径
    realsense_launch_dir = os.path.join(get_package_share_directory('realsense2_camera'), 'launch')
    # 获取你自己功能包里的 yaml 路径
    config_path = os.path.join(
        get_package_share_directory('camera'),
        'config',
        'd435i_params.yaml'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(realsense_launch_dir, 'rs_launch.py')
            ),
            # 2. 通过向 rs_launch 传递 'ros_params_file' 参数来加载 YAML
            launch_arguments={
                'ros_params_file': config_path,
                'enable_gyro': 'true',          # 强制开启陀螺仪
                'enable_accel': 'true',         # 强制开启加速度计
                'unite_imu_method': '2',        # 强制合并 IMU 话题
                'enable_sync': 'true',          # 强制开启 RGB/Depth 同步
                'camera_name': 'camera',        # 确保节点名与 YAML 匹配
                'namespace': 'camera',          # 确保命名空间一致
                # 重定位
                'align_depth.enable': 'true',  # 必须，确保话题为 aligned_depth_to_color
                'enable_sync': 'true',         # 必须，确保 RGB 和 Depth 时间戳同步
                'global_time_enabled': 'true',  # 强制使用系统时间戳
                'initial_reset': 'true',        # 启动时重置相机，清除硬件缓存
            }.items()
        )
    ])