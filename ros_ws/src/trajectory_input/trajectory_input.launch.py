from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='trajectory_input',
            executable='trajectory_input_node',
            name='trajectory_input_node',
            output='screen',
            emulate_tty=True,
        ),
    ])
