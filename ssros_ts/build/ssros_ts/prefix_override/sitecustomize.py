import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ssignal/work/ros2Test/ssros_ts/install/ssros_ts'
