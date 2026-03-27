import os
from setuptools import setup
from glob import glob

package_name = 'my_robot_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装启动文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # 安装URDF文件
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        # 安装RViz配置（先创建config目录）
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ROS 2 Humble robot simulation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 机器人控制节点
            'balance_control_node = my_robot_sim.balance_control_node:main',
        ],
    },
)