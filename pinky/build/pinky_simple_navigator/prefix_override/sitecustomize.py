import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jinhyuk2me/ros_ws/pinky/install/pinky_simple_navigator'
