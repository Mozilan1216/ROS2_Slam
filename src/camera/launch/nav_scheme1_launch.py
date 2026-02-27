import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    camera_pkg = get_package_share_directory('camera')
    
    # 数据库路径
    rtabmap_db_file = '/home/mozilan/nav2_ws/src/camera/rtabmap/rtabmap_20260112.db'
    
    # 核心修改：指向我们将要新建的专属参数文件
    nav2_params_file = os.path.join(camera_pkg, 'config', 'nav2_params_scheme1.yaml')
    rtabmap_config_path = os.path.join(camera_pkg, 'config', 'rtabmap_params_localization.yaml')
    rviz_config_path = os.path.join(camera_pkg, 'rviz', 'navi.rviz') 

    # 1. 启动 Realsense 相机
    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')),
        launch_arguments={
            'align_depth.enable': 'true',
            'enable_sync': 'true',
        }.items()
    )

    # 2. 静态 TF：补全 base_link -> camera_link
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link']
    )

    # 3. RTAB-Map 定位与地图发布 (全面接管)
    rtabmap_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py')),
        launch_arguments={
            'localization': 'true',
            'database_path': rtabmap_db_file,
            'rtabmap_params_proxy': rtabmap_config_path,
            'frame_id': 'base_link',            # 统一为 base_link
            'visual_odometry': 'true',          # 开启视觉里程计
            'publish_tf_odom': 'true',
            'approx_sync': 'true',
            'rgb_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'imu_topic': '/camera/camera/imu',
            'qos': '1',
            # 关键点：让 RTAB-Map 将 2D 栅格地图直接发布到 /map 话题
            'map_topic': '/map',
        }.items()
    )

    # 4. Nav2 纯导航堆栈 (使用 navigation_launch.py 剔除 AMCL 和 Map_Server)
    nav2_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params_file,
            'autostart': 'true',
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        # 1. 添加这一行：强制 OpenGL 版本修复 RViz 报错，解决地图不显示的bug
        SetEnvironmentVariable('MESA_GL_VERSION_OVERRIDE', '3.3'),

        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        camera_node,
        static_tf,
        rviz_node,
        # 阶梯启动：等待相机和 RTAB-Map 稳定并发布地图后，再启动 Nav2
        TimerAction(period=4.0, actions=[rtabmap_node]),
        TimerAction(period=10.0, actions=[nav2_node])
    ])