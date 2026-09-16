import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/kriso/rosetta_drone_ws/rosettadrone_work/install/rd_bridge'
