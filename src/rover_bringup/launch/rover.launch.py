from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rover_odometry',
            executable='odometry_node',
            name='rover_odometry',
            output='screen',
            parameters=[{
                'ticks_per_rev': 360.0,
                'wheel_diameter_m': 0.070,
                'wheel_base_m': 0.142,
                'publish_tf': True,
                'update_hz': 20.0,
            }]
        ),
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{'port': 9090}]
        ),
    ])
