import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/abanoub/delivery_bot_ws/install/twist_to_stamped'
