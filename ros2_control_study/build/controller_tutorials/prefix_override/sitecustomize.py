import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jinhyuk2me/ros_ws/ros2_control_study/install/controller_tutorials'
