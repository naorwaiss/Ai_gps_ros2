from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare configuration arguments
    fcu_url = DeclareLaunchArgument(
        'fcu_url', default_value='serial:///dev/ttyAMA0',
        description='FCU URL for MAVROS'
    )
    gcs_url = DeclareLaunchArgument(
        'gcs_url', default_value='udp://@127.0.0.1',
        description='GCS URL for MAVROS'
    )

    # Start MAVROS node using ExecuteProcess with dynamic command generation
    mavros_process = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'mavros', 'mavros_node', '--ros-args',
            '-p', 'fcu_url:=serial:///dev/ttyAMA0',
            '-p', 'gcs_url:=udp://@127.0.0.1'
        ],
        output='screen'
    )

    # Execute pose_converter node
    pose_converter_process = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'drone_project', 'pose_converter'
        ],
        #output='screen'
    )

    # Execute esc_data_processor node
    esc_data_processor_process = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'drone_project', 'esc_data_processor'
        ],
        #output='screen'
    )

    # Node for v4l2_camera
    v4l2_camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera_node',
        parameters=[{'image_size': [640, 480]}],  # Set parameters here
        #output='screen'
    )

    # Node for optical_flow_processor
    optical_flow_processor_node = Node(
        package='drone_project',
        executable='optical_flow_processor',
        name='optical_flow_processor',
        #output='screen'
    )

    # Launch description to run MAVROS, pose_converter, esc_data_processor, v4l2_camera_node, and optical_flow_processor_node
    return LaunchDescription([
        fcu_url,
        gcs_url,
        mavros_process,
        pose_converter_process,
        esc_data_processor_process,
        v4l2_camera_node,
        optical_flow_processor_node
    ])
