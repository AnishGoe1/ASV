import sys
if sys.prefix == '/home/project/ASV/vir_env_main':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/project/ASV/ASV_new/install/imu'
