import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/determ/MultiCameraMakerTracking/ros2_ws/install/multi_camera_robot_nav'
