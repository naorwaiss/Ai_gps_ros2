from setuptools import setup
import os

package_name = 'drone_project'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/optical_flow_processor.launch.py']),
	 (os.path.join('share', package_name, 'launch'), ['launch/drone_launch.py']), 
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'opencv-python',  # Ensure opencv-python is installed in your environment
        'cv-bridge',  # Make sure cv-bridge is available for ROS2
        'rclpy'  # ROS 2 client library for Python
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='A package for processing optical flow and ESC telemetry in drones',
    license='Apache 2.0',  # Update this line if you have a different license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'optical_flow_processor = drone_project.optical_flow_processor:main',
            'esc_data_processor = drone_project.esc_data_processor:main',
            'pose_converter = drone_project.pose_converter:main',
            'vector_nevigation = drone_project.vector_nevigation:main',
            'get_data = drone_project.get_data:main',
        ],
    },
)
