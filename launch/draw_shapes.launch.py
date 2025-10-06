from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start turtlesim
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),

        # Start your commander node
        Node(
            package='my_robot_controller',
            executable='turtle_commander',
            name='turtle_commander',
            output='screen'
        ),
    ])
