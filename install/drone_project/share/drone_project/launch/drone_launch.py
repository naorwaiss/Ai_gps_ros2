from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare configuration arguments
    fcu_url = DeclareLaunchArgument(
        'fcu_url', default_value='udp://127.0.0.1:14540@14557',
        #        'fcu_url', default_value='serial:///dev/ttyAMA0',

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
            '-p', 'fcu_url:=udp://127.0.0.1:14540@14557',
            '-p', 'gcs_url:=udp://@127.0.0.1'
        ],
        output='screen'
    )

    # Execute pose_converter node
    pose_converter_process = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'drone_project', 'pose_converter'
        ],
        output='screen'
    )

    # Execute esc_data_processor node
    esc_data_processor_process = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'drone_project', 'esc_data_processor'
        ],
        output='screen'
    )

    # Launch description to run MAVROS, pose_converter, and esc_data_processor
    return LaunchDescription([
        fcu_url,
        gcs_url,
        mavros_process,
        pose_converter_process,
        esc_data_processor_process
    ])
