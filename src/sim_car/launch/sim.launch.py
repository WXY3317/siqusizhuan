from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from sim_car.model import generate

def generate_launch_description():
    share=Path(get_package_share_directory('sim_car'))
    rviz_node = Node(package='rviz2', executable='rviz2',
                     arguments=['-d', str(share/'rviz/poseidon.rviz')],
                     condition=IfCondition(LaunchConfiguration('rviz')))
    return LaunchDescription([
        DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('demo',default_value='false'),
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':generate()}]),
        Node(package='sim_car',executable='simulator',parameters=[{'demo':ParameterValue(LaunchConfiguration('demo'),value_type=bool)}]),
        RegisterEventHandler(OnProcessExit(
            target_action=rviz_node,
            on_exit=[EmitEvent(event=Shutdown(reason='RViz closed'))])),
        rviz_node
    ])
