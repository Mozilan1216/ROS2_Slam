1. 检测摄像头驱动代码
ros2 launch d435i_launch.py
// 重定位代码的驱动是直接调用的该代码

验证陀螺仪状态（应返回 Boolean value is: True）：
ros2 param get /camera/camera enable_gyro

2. D435i 进行视觉建模
ros2 launch slam_launch.py


3. 根据 D435i 建好的地图进行 nav 导航
ros2 launch nav_d435i_launch.py
