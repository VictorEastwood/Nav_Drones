from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    nav_drone_control_node = Node(
        package='nav_drone_control',
        executable='nav_drone_control_node',
        output='screen',
        shell=True,
    )

    return LaunchDescription([
        nav_drone_control_node
    ])