import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/tuf/Mark/face_robot_ws/install/face_robot_driver'
