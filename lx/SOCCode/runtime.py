#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""运行时角色与环境配置。

根据命令行 --role 参数或 NANO_ROLE 环境变量决定本机角色（primary/backup），
并暴露 IP、端口等全局常量供心跳等模块使用。
"""

from config import resolve_runtime_config


_runtime_config = resolve_runtime_config()

NANO_ROLE = _runtime_config.nano_role         # 当前角色字符串 'primary' 或 'backup'
IS_PRIMARY = _runtime_config.is_primary       # 是否为主机
PRIMARY_IP = _runtime_config.primary_ip       # 主机 IP 地址
SECONDARY_IP = _runtime_config.secondary_ip   # 备机 IP 地址
HB_PORT = _runtime_config.hb_port             # 心跳 UDP 端口
HB_GRACE_S = _runtime_config.hb_grace_s       # 心跳启动宽限时间 (s)
LOG_FILE = _runtime_config.log_file           # 日志文件路径


def configure_runtime(role=None):
    """重新配置运行时角色，在命令行解析后调用以覆盖默认值。"""
    global _runtime_config, NANO_ROLE, IS_PRIMARY, PRIMARY_IP, SECONDARY_IP, HB_PORT, HB_GRACE_S, LOG_FILE

    _runtime_config = resolve_runtime_config(role)
    NANO_ROLE = _runtime_config.nano_role
    IS_PRIMARY = _runtime_config.is_primary
    PRIMARY_IP = _runtime_config.primary_ip
    SECONDARY_IP = _runtime_config.secondary_ip
    HB_PORT = _runtime_config.hb_port
    HB_GRACE_S = _runtime_config.hb_grace_s
    LOG_FILE = _runtime_config.log_file
    return _runtime_config


def get_runtime_config():
    """返回当前运行时配置对象。"""
    return _runtime_config