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
                'ros_params_file': config_path
            }.items()
        )
    ])