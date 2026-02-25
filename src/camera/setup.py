from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'map'), glob('map/*')),
        (os.path.join('share', package_name, 'rtabmap'), glob('rtabmap/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mozilan',
    maintainer_email='mozilan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vision_node = camera.vision_node:main',    # 视觉检测代码
            'semantic_nav = camera.semantic_nav:main', # 语义导航代码
            'get_coords = camera.get_coordsmain', # 获取坐标代码
            'semantic_map= camera.semantic_map:main',
            'semantic_detector= camera.semantic_detector:main',
            'snav_to_object.py= camera.nav_to_object.py:main',
            
        ],
    },
)
