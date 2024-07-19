import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/naor/Desktop/naor/study/Ai_gps_ros2/install/drone_project'
