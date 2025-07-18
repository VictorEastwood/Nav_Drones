from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    micro_XRCE_agent_serial = ExecuteProcess(
        cmd=[[
            'sudo MicroXRCEAgent serial --dev /dev/ttyTHS1 -b 921600'
        ]],
        shell=True
    )

    nav_drone_control_node = Node(
        package='nav_drone_control',
        executable='nav_drone_control_node',
        output='screen',
        shell=True,
    )

    return LaunchDescription([
        # micro_XRCE_agent_serial,
        nav_drone_control_node
    ])