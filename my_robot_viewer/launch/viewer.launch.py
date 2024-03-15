from launch import LaunchDescription
from launch_ros.actions import Node


 
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_viewer',
            executable='mqtt_ros_bridge',
            name='bridge_node',
            output='screen',
        ),
        Node(
            package='my_robot_viewer',
            executable='sub_node',
            name='stream_subscriber',
            output='screen',
        ),
    ])
 