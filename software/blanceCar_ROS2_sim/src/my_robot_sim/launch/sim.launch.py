import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler,TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart, OnProcessExit

def generate_launch_description():
    # 1. 获取功能包路径
    pkg_share = get_package_share_directory('my_robot_sim')
    
    # 2. 声明仿真时间参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # 3. 处理XACRO文件，生成URDF内容
    robot_description_content = Command(
        [
            'xacro ',
            os.path.join(pkg_share, 'urdf', 'robot.xacro')
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    # 4. 启动Gazebo空世界
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time, 'world_name': os.path.join(pkg_share, 'worlds', 'my_ground.world')}.items()
    )
    
    # 5. 启动Gazebo客户端
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        )
    )

    # 6. 机器人状态发布器节点（负责TF变换生成）
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            robot_description, 
            {'use_sim_time': use_sim_time},
            {'publish_frequency': 50.0}  # 设置发布频率
        ],
        output='both'  # 同时输出到屏幕和日志文件
    )

    # 7. 在Gazebo中生成机器人模型（抬高z轴避免陷地）
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_robot',
            '-x', '0.0', '-y', '0.0', '-z', '0.15'  # 调整高度，使车轮刚好接触地面
        ],
        output='screen'
    )

    # 8. 启动RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'config', 'rviz_config.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=None
    )

    # 9. 启动平衡控制节点
    balance_control_node = Node(
        package='my_robot_sim',
        executable='balance_control_node',
        name='balance_control_node',
        output='screen'
    )

    # 组装所有启动项
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time if true'),
        gazebo,
        gazebo_client,
        robot_state_publisher,
        spawn_entity,  # 先在Gazebo中生成机器人模型
        rviz2,
        balance_control_node
    ])