# 共享桥模块路径：复用 HIL/carla_bridge/pc/ 下的 bridge_config / scenarios / carla_link
import os, sys
_HERE = os.path.dirname(os.path.abspath(__file__))
_BRIDGE = os.path.normpath(os.path.join(_HERE, '..', 'HIL', 'carla_bridge', 'pc'))
if _BRIDGE not in sys.path:
    sys.path.insert(0, _BRIDGE)
