# 🚗 萌驾舱 MoeDrive · 统一可爱演示网站

把【实时驾驶舱】+【AI 智能监控】+【边缘计算】+【项目报告】整合进**同一个网站**，
统一可爱（pastel / 圆角 / 吉祥物）风格。纯 Python 标准库，**零第三方依赖、零 CDN**，
无需 CARLA、无需 Ollama 也能完整演示。

## 一键启动

```powershell
双击  启动.bat
# 或
python server.py            # 浏览器自动开 http://127.0.0.1:8099
python server.py --speed 4  # 4 倍速演示（录制回溯默认 1x 真实时间）
python server.py --no-scenarios   # 关闭录制场景回溯，改用内置脚本仿真
python server.py --no-browser --port 8099
```

## 五个页签（一个网站，导航切换）

| 页签 | 内容 |
|---|---|
| 🏠 总览 | 项目标题 / 简介 + 6 张关键指标卡 + 实时模式速度 + 一键直达 |
| 🚗 实时驾驶舱 | 车速 / 模式 / 功能 chips / 前车跟踪 / 车道可视化 / 执行器 / 主备控制源 / 告警 / 趋势曲线 |
| 🧠 AI 智能监控 | 边缘 KPI 喂给 AI → 风险评估 + 关键发现 + 调参建议（吉祥物气泡）+ 分析历史 |
| 📊 边缘计算 | 5s 滑窗 KPI 网格 + 事件流 + 上云条数 |
| 📖 项目报告 | 功能需求 FR / 非功能需求 NFR / 八项创新点卡片（摘自竞赛报告） |

## 数据来源（三个后端都能接）

- **默认 · 历史数据仓库回溯**：`数据仓库/` 有 `index.json` 时自动进入回溯模式——
  「🚗 实时驾驶舱」顶部出现「🗂️ 历史数据仓库」面板，**按场景类型归类**（超车/加塞/行人/
  ACC/超高速/巡航…）、组内**按日期倒序**，每条记录显示**关键 KPI 摘要**（最高速/最小车距/
  最小 TTC/急刹·AEB 次数）+ **风险等级色卡**；点任一条即 **1:1 真实时间**回放，驾驶舱 +
  边缘计算 + AI 全部实时联动。仓库可持续积累，归档/重建见下方「数据仓库」。`--no-scenarios` 关闭。
- **自包含脚本仿真**（无仓库索引或 `--no-scenarios`）：进程内 `sim_feed`（66s 七场景脚本）+
  `edge`（边缘计算）+ `ai_analyzer`（规则引擎，检测到本地 Ollama qwen2.5:3b 则自动增强）。

### 🗂️ 数据仓库（历史 CSV 归类 / 总结 / 回放）

```powershell
# 把任意历史运行 CSV 归档进仓库（自动归类 + 算 KPI 摘要 + 风险，并重建索引）
python archive_csv.py 某次运行.csv
python archive_csv.py a.csv b.csv --category acc --name 雨天跟车
python archive_csv.py --from-logs ..\仿真\logs                         # 批量导入目录
python archive_csv.py --rebuild           # 仅重建索引
python repo_index.py                       # 同上，重建索引并打印归类总结
```
仓库目录 `数据仓库/`：放 CSV + 自动生成 `index.json`。归类与 KPI 口径同网站边缘计算
（全程不滑窗聚合）。文件名 `NN_中文名.csv` 时按名归类，否则按数据特征推断。
- **代理真实 ADAS 后端（HTTP）**：`python server.py --adas-url http://127.0.0.1:8088`，
  实时数据改取自 4070 演示包的 `run_adas.py` / `web_demo.py` 后端。
- **MQTT 订阅（局域网 / 跨机推荐）**：`python server.py --mqtt-broker`，
  在本机内置一个**纯标准库 MQTT broker（零安装）**并订阅 `adas/state`；ADAS 机用
  `run_adas.py --mqtt --mqtt-host <网站机IP>` / `web_demo.py --mqtt ...` 把每帧状态
  推过来即在仪表盘显示。已有 mosquitto 时改用 `--mqtt-host 192.168.x.x` 指过去即可。
  详见 4070 演示包内 `ADAS_Central/README_MQTT上报.md`。

  | 参数 | 默认 | 说明 |
  |---|---|---|
  | `--mqtt-host` | 无 | 订阅的 broker 地址（设了才进 MQTT 模式） |
  | `--mqtt-port` | `1883` | broker 端口 |
  | `--mqtt-topic` | `adas/state` | 订阅主题 |
  | `--mqtt-broker` | 关 | 在本机内置纯标准库 broker（并默认订阅 127.0.0.1） |

## 文件

| 文件 | 作用 |
|---|---|
| `server.py` | 统一服务（聚合 + SSE + 报告 API + 静态；支持 MQTT 订阅 / 内置 broker） |
| `mqtt_lite.py` | 纯标准库 MQTT 3.1.1（QoS0）客户端 + 极简 broker（零 pip；`--selftest` 自检） |
| `adas_core.py` | 内置脚本仿真核心（LKA/ACC/AEB/行人/超车/主备接管，自包含） |
| `edge_engine.py` | 边缘计算引擎（5s 滑窗 KPI / 事件 / 风险 / 上云，自包含） |
| `csv_replay.py` | 录制场景 CSV 回放器（CSV 行 → 驾驶舱帧） |
| `录制场景CSV/` | 各场景真实运行 CSV（回溯数据源，可用 export_scenarios_csv.py 刷新） |
| `sim_feed.py` / `edge.py` | 兼容转发层（转发到本地 `adas_core` / `edge_engine`） |
| `ai_analyzer.py` | AI 分析（规则引擎 + 可选 Ollama） |
| `report_highlights.json` | 报告亮点（由 `extract_report.py` 从 docx 提取后人工精炼） |
| `extract_report.py` | 从 `unpacked_report2` 重新提取报告内容 |
| `web/` | 可爱风格单页前端（index.html / style.css / app.js） |
| `绘图/plot_effects.py` | 跑通管线并出「新功能效果数据图」PNG |

## 环境变量

`OLLAMA_URL`（默认 `http://127.0.0.1:11434`）、`OLLAMA_MODEL`（默认 `qwen2.5:3b`）。
Ollama 未启动时 AI 监控自动回退到内置规则引擎，其余功能不受影响。
