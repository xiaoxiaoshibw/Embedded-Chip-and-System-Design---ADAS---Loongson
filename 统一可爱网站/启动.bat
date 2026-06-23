@echo off
chcp 65001 >nul 2>&1
title 萌驾舱 MoeDrive - 统一可爱演示网站
rem 一键启动统一网站（无需 CARLA / 无需 Ollama 也能完整演示）。
rem 浏览器自动打开 http://127.0.0.1:8099
rem 接真实 ADAS 后端：  启动.bat --adas-url http://127.0.0.1:8088
python "%~dp0server.py" %*
pause
