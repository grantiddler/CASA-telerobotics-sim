import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/grant/CASA-telerobotics-sim/install/telerobotics_sim'
