# -*- coding: utf-8 -*-
"""ADAS HIL 平台核心层。

设计原则（与需求一致）：
- 底层 CARLA/Nano/ESP32 的控制权**只由 SimulationCore 统一持有**，CLI / Web /
  REST 都通过 SimulationCore，绝不直接 tick CARLA、绝不多处持有 actor。
- 本层为纯 Python（不依赖 FastAPI），便于单独跑通和单元自检。
- 真实硬件接入通过 HilBridge 适配器，默认提供 MockHilBridge（无 CARLA/Nano/ESP32
  也能演示），后续可替换为接 `carla_bridge/` 真实链路的实现，无需改上层。
"""

from .state_machine import SimState, StateMachine  # noqa: F401
from .simulation_core import SimulationCore  # noqa: F401
