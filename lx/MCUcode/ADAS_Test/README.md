# ADAS AEB (Automatic Emergency Braking) Control System

这是一个ESP32/ESP-IDF项目，实现自动紧急制动（AEB）系统。

## 项目功能

- **UART通信**：与Jetson平台通过UART1通信，接收以下数据：
  - TTC (Time-To-Collision): 碰撞时间
  - DIST: 与前车距离
  - PSI: 转向角度
  - DELTA: 油门角度
  - SPEED: 车速

- **AEB算法**：
  - TTC < TTC_THRESHOLD (3.5s) 时触发渐进式制动
  - 距离 < DANGER_DIST (20.0m) 时触发最大制动
  - 输出制动加速度 (a_brake)

- **输出格式**：固定长度格式 `+X.XXXX,+X.XX,X.XX`
  - 字段1: PSI (±9.9999)
  - 字段2: DELTA (±9.99)
  - 字段3: 制动加速度 (0.00-9.99)

## 硬件引脚

- RX_PIN = GPIO 16
- TX_PIN = GPIO 17
- BAUDRATE = 115200

## 编译

```bash
idf.py build
```

## 烧录

```bash
idf.py flash
```

## 监控

```bash
idf.py monitor
```

## 完整构建和烧录

```bash
idf.py build flash monitor
```
