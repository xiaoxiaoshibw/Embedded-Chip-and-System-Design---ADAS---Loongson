# -*- coding: utf-8 -*-
"""FastAPI 请求/响应 Pydantic 模型。

只对"写"接口做严格校验；状态/指标/回放等读接口直接返回 dict，避免与核心层
数据结构强耦合（核心层已自带空值保护与序列化）。
"""

from __future__ import annotations

from typing import Any, Dict, Optional

from pydantic import BaseModel, Field


class LoadScenarioRequest(BaseModel):
    scenario: str = Field(..., description="场景名，如 acc_follow / takeover")
    params: Optional[Dict[str, Any]] = Field(
        default=None, description="覆盖场景默认参数（可选）")


class UpdateParametersRequest(BaseModel):
    params: Dict[str, Any] = Field(..., description="要更新的参数键值")


class InjectFaultRequest(BaseModel):
    fault_type: str = Field(..., description="seq_stuck/heartbeat_loss/nan_output/control_delay/backup_fail/dual_fail")
    target: str = Field(default="nano_a", description="nano_a / nano_b / both")


class WorldWeatherRequest(BaseModel):
    weather: str = Field(..., description="clear / rain / fog / night")


class WorldNpcRequest(BaseModel):
    count: int = Field(default=5, ge=0, le=80, description="生成 NPC 车辆数")


class WorldLeadRequest(BaseModel):
    kmh: Optional[float] = Field(default=None, description="前车目标速度 km/h，null=恢复场景脚本")


class WorldManualRequest(BaseModel):
    on: bool = Field(..., description="是否开启手动驾驶接管")


class WorldManualCmdRequest(BaseModel):
    throttle: float = 0.0
    brake: float = 0.0
    steer: float = 0.0


class HardwareRestartRequest(BaseModel):
    target: str = Field(default="both", description="primary / backup / both")


class HardwareGatewayRequest(BaseModel):
    source: str = Field(default="esp32", description="esp32 / jetson")


class ActionResponse(BaseModel):
    ok: bool = True
    status: Dict[str, Any]


class SimpleOk(BaseModel):
    ok: bool = True
    detail: Optional[Dict[str, Any]] = None
