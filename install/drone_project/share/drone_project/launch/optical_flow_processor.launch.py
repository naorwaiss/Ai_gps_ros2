from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the v4l2_camera_node
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera_node',
            parameters=[{'image_size': [640, 480]}],  # Set parameters here
            output='screen'
        ),
        # Launch the optical_flow_processor node
        Node(
            package='drone_project',
            executable='optical_flow_processor',
            name='optical_flow_processor',
            output='screen'
        )
    ])
