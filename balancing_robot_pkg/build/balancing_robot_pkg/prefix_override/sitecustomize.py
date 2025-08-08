import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/fayas/main_project/balancing_robot_pkg/install/balancing_robot_pkg'
