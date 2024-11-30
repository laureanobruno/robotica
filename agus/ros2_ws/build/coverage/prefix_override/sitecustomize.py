import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/apoire/Documents/Robotica/ros2_ws/install/coverage'
