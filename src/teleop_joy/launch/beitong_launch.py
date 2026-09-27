import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import uuid

def generate_launch_description():

    usr_task = LaunchConfiguration('usr_task')
    x_speed_scale = LaunchConfiguration('x_speed_scale')
    y_speed_scale = LaunchConfiguration('y_speed_scale')
    w_speed_scale = LaunchConfiguration('w_speed_scale')
    task_a = LaunchConfiguration('task_a')
    task_b = LaunchConfiguration('task_b')
    steer_gain = LaunchConfiguration('steer_gain')
    udp_send_on_connect = LaunchConfiguration('udp_send_on_connect')

    # 启动 ROS2 手柄驱动节点（joy_node）
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[
            {"dev": "/dev/input/js0"},       # 如需其他手柄设备路径可在运行时重映射
            {"deadzone": 0.05},
            {"autorepeat_rate": 20.0}
        ]
    )

    # 定义手柄解析节点
    teleop_joy_node = Node(
        package="teleop_joy",           # 功能包名
        executable="teleop_joy",        # 可执行文件名称
        output="screen",               # 日志输出到终端
        parameters=[                   # 可选：添加节点参数
            # config_file,              
            {"usr_task": 0},  
            {"x_speed_scale": 0.7},         
            {"y_speed_scale": 0.3},
            {"w_speed_scale": 0.3},
            {"task_a": 1},
            {"task_b": 2},  
            {"steer_gain": 50},   
        ],
        #重映射话题
        remappings=[
            # ("wheels_raw_data", "/sensor/wheels/raw"),    # 原始数据话题重映射
            ("cmd_vel", "cmd_vel")   # 解析cmd_vel话题重映射
        ]
    )

    # 组装 launch 描述
    return LaunchDescription([
        # 如果直接运行该脚本，也可声明参数的默认值
        DeclareLaunchArgument('usr_task', default_value='0', description='usr task selection'),
        DeclareLaunchArgument('x_speed_scale', default_value='0.7', description='X speed scale factor'),
        DeclareLaunchArgument('y_speed_scale', default_value='0.3', description='Y speed scale factor'),
        DeclareLaunchArgument('w_speed_scale', default_value='0.3', description='W speed scale factor'),
        DeclareLaunchArgument('task_a', default_value='1', description='Task A selection'),
        DeclareLaunchArgument('task_b', default_value='2', description='Task B selection'),
        DeclareLaunchArgument('steer_gain', default_value='50', description='Steer gain factor'),
        joy_node,
        teleop_joy_node
    ])