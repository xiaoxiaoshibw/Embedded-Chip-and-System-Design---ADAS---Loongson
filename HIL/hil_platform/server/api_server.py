# -*- coding: utf-8 -*-
"""统一 Simulation API Server（FastAPI）。

CLI 和两个 Web 页面（/live、/replay）都只调用这里的 API；本进程持有**唯一**的
SimulationCore 实例，是底层 CARLA/Nano/ESP32 的唯一控制入口。

- 实时控制接口：可改变仿真（load/start/pause/stop/reset/parameters/fault）。
- 历史回放接口：只读 runs/ 目录，绝不触碰 CARLA / 仿真状态。
- /ws/live：每 100ms 推送一帧最新状态。

启动：
    python -m server.api_server                 # mock 模式（默认）
    uvicorn server.api_server:app --reload      # 开发
环境变量：
    HIL_MOCK=0     关闭 mock（接真实 CARLA，未实现前会报错）
    HIL_PORT=8000  端口
"""

from __future__ import annotations

import asyncio
import csv
import json
import os
from typing import Any, Dict, List, Optional

from fastapi import FastAPI, HTTPException, Query, Response, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, HTMLResponse, PlainTextResponse

from core.recorder import RUNS_DIR, STATE_FIELDS
from core.scenario_manager import ScenarioManager
from core.simulation_core import SimulationCore
from core.state_machine import InvalidTransition
from core import hardware_control
from server.schemas import (
    HardwareGatewayRequest,
    HardwareRestartRequest,
    InjectFaultRequest,
    LoadScenarioRequest,
    UpdateParametersRequest,
    WorldLeadRequest,
    WorldManualCmdRequest,
    WorldManualRequest,
    WorldNpcRequest,
    WorldWeatherRequest,
)

# ── 唯一核心实例 ──
_MOCK = os.environ.get("HIL_MOCK", "1") != "0"
core = SimulationCore(mock=_MOCK)

app = FastAPI(title="ADAS HIL Platform API", version="0.1.0")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],          # 开发期放开；生产可收紧到前端域名
    allow_methods=["*"],
    allow_headers=["*"],
)

# 整数字段（其余数值按 float 解析）
_INT_FIELDS = {
    "nano_a_seq", "nano_a_alive", "nano_a_valid_output",
    "nano_b_seq", "nano_b_alive", "nano_b_valid_output",
    "takeover_count", "safe_brake",
}
_STR_FIELDS = {"active_controller", "event"}


def _guard(fn):
    """把核心层异常转成合适的 HTTP 状态码。"""
    try:
        return fn()
    except InvalidTransition as exc:
        raise HTTPException(status_code=409, detail=str(exc))
    except (RuntimeError, ValueError, KeyError) as exc:
        raise HTTPException(status_code=400, detail=str(exc))
    except FileNotFoundError as exc:
        raise HTTPException(status_code=404, detail=str(exc))


# ══════════════════════════════════════════════════════════════
# 实时控制接口
# ══════════════════════════════════════════════════════════════
@app.post("/api/scenario/load")
def scenario_load(req: LoadScenarioRequest):
    return _guard(lambda: {"ok": True, "status": core.load_scenario(req.scenario, req.params)})


@app.post("/api/simulation/start")
def simulation_start():
    return _guard(lambda: {"ok": True, "status": core.start()})


@app.post("/api/simulation/pause")
def simulation_pause():
    return _guard(lambda: {"ok": True, "status": core.pause()})


@app.post("/api/simulation/stop")
def simulation_stop():
    return _guard(lambda: {"ok": True, "status": core.stop(), "meta": core.last_meta})


@app.post("/api/simulation/reset")
def simulation_reset():
    return _guard(lambda: {"ok": True, "status": core.reset()})


@app.post("/api/parameters/update")
def parameters_update(req: UpdateParametersRequest):
    return _guard(lambda: {"ok": True, **core.update_parameters(req.params)})


@app.post("/api/fault/inject")
def fault_inject(req: InjectFaultRequest):
    return _guard(lambda: {"ok": True, **core.inject_fault(req.fault_type, req.target)})


@app.get("/api/status")
def get_status():
    return core.status()


@app.get("/api/metrics")
def get_metrics():
    return core.metrics_snapshot()


@app.get("/api/hardware/health")
def hardware_health():
    return _guard(lambda: hardware_control.health())


@app.post("/api/hardware/adas/restart")
def hardware_restart_adas(req: HardwareRestartRequest):
    return _guard(lambda: hardware_control.restart_adas(req.target))


@app.post("/api/hardware/gateway/start")
def hardware_start_gateway(req: HardwareGatewayRequest):
    return _guard(lambda: hardware_control.start_gateway(req.source))


@app.post("/api/hardware/nanos/restore")
def hardware_restore_nanos():
    return _guard(lambda: hardware_control.restore_nanos())


# ── 自由操控世界（仅真实 CARLA 模式有效）──
@app.post("/api/world/weather")
def world_weather(req: WorldWeatherRequest):
    return _guard(lambda: {"ok": True, **core.world_command("weather", weather=req.weather)})


@app.post("/api/world/npc")
def world_npc(req: WorldNpcRequest):
    return _guard(lambda: {"ok": True, **core.world_command("spawn_npc", count=req.count)})


@app.post("/api/world/npc/clear")
def world_npc_clear():
    return _guard(lambda: {"ok": True, **core.world_command("clear_npc")})


@app.post("/api/world/lead_speed")
def world_lead_speed(req: WorldLeadRequest):
    return _guard(lambda: {"ok": True, **core.world_command("lead_speed", kmh=req.kmh)})


@app.post("/api/world/manual")
def world_manual(req: WorldManualRequest):
    return _guard(lambda: {"ok": True, **core.world_command("manual", on=req.on)})


@app.post("/api/world/manual_cmd")
def world_manual_cmd(req: WorldManualCmdRequest):
    return _guard(lambda: {"ok": True, **core.world_command(
        "manual_cmd", throttle=req.throttle, brake=req.brake, steer=req.steer)})


@app.get("/api/world/camera")
def world_camera():
    # 优先内存 JPEG（绕开中文路径），无则回退 ASCII 临时 PNG
    jpeg = core.camera_jpeg()
    if jpeg:
        return Response(content=jpeg, media_type="image/jpeg",
                        headers={"Cache-Control": "no-store"})
    p = core.camera_path()
    if p:
        return FileResponse(p, media_type="image/png", headers={"Cache-Control": "no-store"})
    raise HTTPException(status_code=404, detail="无摄像头帧（mock 模式或 CARLA 未就绪）")


@app.get("/api/scenarios")
def get_scenarios():
    """列出可用场景及默认参数（前端 Load 面板用，非强制需求接口）。"""
    mgr = ScenarioManager()
    out = []
    for name in mgr.list_scenarios():
        try:
            out.append(mgr.load(name).to_dict())
        except Exception:
            out.append({"name": name})
    return {"scenarios": out}


# ══════════════════════════════════════════════════════════════
# 历史回放接口（只读 runs/）
# ══════════════════════════════════════════════════════════════
def _run_dir(run_id: str) -> str:
    # 防目录穿越
    if "/" in run_id or "\\" in run_id or run_id in ("", ".", ".."):
        raise HTTPException(status_code=400, detail="非法 run_id")
    d = os.path.join(RUNS_DIR, run_id)
    if not os.path.isdir(d):
        raise HTTPException(status_code=404, detail="run 不存在：%s" % run_id)
    return d


def _read_json(path: str) -> Any:
    if not os.path.isfile(path):
        raise HTTPException(status_code=404, detail="缺少文件：%s" % os.path.basename(path))
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)


def _parse_states(path: str, stride: int = 1) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    if not os.path.isfile(path):
        return rows
    with open(path, "r", encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        for i, raw in enumerate(reader):
            if stride > 1 and (i % stride != 0):
                continue
            row: Dict[str, Any] = {}
            for k, v in raw.items():
                if k in _STR_FIELDS:
                    row[k] = v
                elif v == "" or v is None:
                    row[k] = None
                elif k in _INT_FIELDS:
                    try:
                        row[k] = int(float(v))
                    except ValueError:
                        row[k] = None
                else:
                    try:
                        row[k] = float(v)
                    except ValueError:
                        row[k] = None
            rows.append(row)
    return rows


@app.get("/api/runs")
def list_runs(
    scenario: Optional[str] = None,
    date: Optional[str] = Query(None, description="YYYYMMDD 前缀"),
    takeover: Optional[bool] = None,
    collision: Optional[bool] = None,
    result: Optional[str] = Query(None, description="PASS / FAIL"),
):
    if not os.path.isdir(RUNS_DIR):
        return {"runs": []}
    items: List[Dict[str, Any]] = []
    for run_id in sorted(os.listdir(RUNS_DIR), reverse=True):
        d = os.path.join(RUNS_DIR, run_id)
        if not os.path.isdir(d):
            continue
        meta_p = os.path.join(d, "meta.json")
        sum_p = os.path.join(d, "summary.json")
        if not os.path.isfile(meta_p):
            continue
        try:
            meta = json.load(open(meta_p, encoding="utf-8"))
            summ = json.load(open(sum_p, encoding="utf-8")) if os.path.isfile(sum_p) else {}
        except (ValueError, OSError):
            continue
        item = {
            "run_id": run_id,
            "start_time": meta.get("start_time"),
            "scenario": meta.get("scenario"),
            "duration": meta.get("duration"),
            "result": summ.get("result"),
            "collision": summ.get("collision"),
            "takeover_happened": summ.get("takeover_happened"),
            "min_ttc": summ.get("min_ttc"),
            "max_lateral_error": summ.get("max_lateral_error"),
        }
        # 过滤
        if scenario and item["scenario"] != scenario:
            continue
        if date and not run_id.startswith(date):
            continue
        if takeover is not None and bool(item["takeover_happened"]) != takeover:
            continue
        if collision is not None and bool(item["collision"]) != collision:
            continue
        if result and (item["result"] or "").upper() != result.upper():
            continue
        items.append(item)
    return {"runs": items}


@app.get("/api/runs/{run_id}/meta")
def run_meta(run_id: str):
    return _read_json(os.path.join(_run_dir(run_id), "meta.json"))


@app.get("/api/runs/{run_id}/summary")
def run_summary(run_id: str):
    return _read_json(os.path.join(_run_dir(run_id), "summary.json"))


@app.get("/api/runs/{run_id}/events")
def run_events(run_id: str):
    return _read_json(os.path.join(_run_dir(run_id), "events.json"))


@app.get("/api/runs/{run_id}/states")
def run_states(run_id: str, stride: int = Query(1, ge=1, le=50)):
    d = _run_dir(run_id)
    return {"fields": STATE_FIELDS, "states": _parse_states(os.path.join(d, "states.csv"), stride)}


@app.get("/api/runs/{run_id}/state")
def run_state_at(run_id: str, t: float = Query(..., description="目标场景时间 s")):
    d = _run_dir(run_id)
    rows = _parse_states(os.path.join(d, "states.csv"))
    if not rows:
        raise HTTPException(status_code=404, detail="无状态数据")
    nearest = min(rows, key=lambda r: abs((r.get("t") or 0.0) - t))
    return nearest


@app.get("/api/runs/{run_id}/report", response_class=PlainTextResponse)
def run_report(run_id: str):
    p = os.path.join(_run_dir(run_id), "report.md")
    if not os.path.isfile(p):
        raise HTTPException(status_code=404, detail="缺少 report.md")
    with open(p, "r", encoding="utf-8") as fh:
        return fh.read()


# ══════════════════════════════════════════════════════════════
# 实时 WebSocket
# ══════════════════════════════════════════════════════════════
@app.websocket("/ws/live")
async def ws_live(ws: WebSocket):
    await ws.accept()
    try:
        while True:
            payload = core.get_live_payload()
            if payload is None:
                # 尚无数据：推一个最小状态帧，前端据此显示"--"而非崩溃
                payload = {"run_id": core.run_id or None, "state": core.sm.state.value,
                           "scenario": core.scenario.name if core.scenario else None,
                           "timestamp": None, "ego": None, "target": None,
                           "nano_a": None, "nano_b": None, "esp32": None, "event": None}
            await ws.send_text(json.dumps(payload, ensure_ascii=False))
            await asyncio.sleep(0.1)   # 100ms / 10Hz 推送
    except WebSocketDisconnect:
        return
    except Exception:
        # 任何异常都安静收尾，避免刷错误日志
        try:
            await ws.close()
        except Exception:
            pass


# ══════════════════════════════════════════════════════════════
# 可选：服务前端构建产物（web/dist），让单进程同时托管 /live、/replay
# ══════════════════════════════════════════════════════════════
_WEB_DIST = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "web", "dist"))


@app.get("/", response_class=HTMLResponse)
def root():
    index = os.path.join(_WEB_DIST, "index.html")
    if os.path.isfile(index):
        with open(index, "r", encoding="utf-8") as fh:
            return fh.read()
    return HTMLResponse(
        "<h3>ADAS HIL Platform API 运行中</h3>"
        "<p>前端尚未构建。开发模式请在 web/ 下 <code>npm install &amp;&amp; npm run dev</code>，"
        "或构建后由本服务托管。</p>"
        "<p>API 文档：<a href='/docs'>/docs</a></p>"
    )


def _mount_spa():
    """若已构建前端，则挂载静态资源并对 /live、/replay 回退到 index.html。"""
    if not os.path.isdir(_WEB_DIST):
        return
    from fastapi.staticfiles import StaticFiles
    assets = os.path.join(_WEB_DIST, "assets")
    if os.path.isdir(assets):
        app.mount("/assets", StaticFiles(directory=assets), name="assets")

    @app.get("/live", response_class=HTMLResponse)
    @app.get("/replay", response_class=HTMLResponse)
    def spa_page():
        with open(os.path.join(_WEB_DIST, "index.html"), "r", encoding="utf-8") as fh:
            return fh.read()


_mount_spa()


def main():
    import uvicorn
    port = int(os.environ.get("HIL_PORT", "8000"))
    uvicorn.run(app, host="0.0.0.0", port=port)


if __name__ == "__main__":
    main()
