@echo off
setlocal EnableExtensions
chcp 65001 >nul

cd /d "%~dp0"

set "ACTUATION_SOURCE=%~1"
if "%ACTUATION_SOURCE%"=="" set "ACTUATION_SOURCE=jetson"

set "SCENARIO=%~2"
if "%SCENARIO%"=="" set "SCENARIO=acc"

echo ============================================================
echo  LAN HIL closed loop launcher
echo  Folder: %CD%
echo  Actuation source: %ACTUATION_SOURCE%
echo  Scenario: %SCENARIO%
echo  Link: TCP 42110 (Windows actively connects to Nano)
echo ============================================================
echo.

echo [1/7] Checking LAN and ROS2 graph...
powershell -NoProfile -ExecutionPolicy Bypass -File "%CD%\check_lan_ros2.ps1"
if errorlevel 1 goto :fail

echo.
echo [2/7] Deploying integrated HIL folder to both Nano boards...
powershell -NoProfile -ExecutionPolicy Bypass -File "%CD%\deploy_gateway.ps1"
if errorlevel 1 goto :fail

echo.
echo [3/7] Stopping old /perception_sim publisher...
powershell -NoProfile -ExecutionPolicy Bypass -File "%CD%\stop_perception_sim_lan.ps1"
if errorlevel 1 goto :fail

echo.
echo [4/7] Starting HIL ADAS nodes on ROS_DOMAIN_ID=43...
powershell -NoProfile -ExecutionPolicy Bypass -File "%CD%\start_hil_adas_lan.ps1"
if errorlevel 1 goto :fail

echo.
echo [5/7] Starting CARLA if needed...
powershell -NoProfile -ExecutionPolicy Bypass -File "%CD%\start_carla_if_needed.ps1"
if errorlevel 1 goto :fail

echo.
echo [6/7] Starting Nano ROS2 gateway in a new window...
start "HIL Nano ROS2 Gateway" /D "%CD%" powershell -NoExit -NoProfile -ExecutionPolicy Bypass -File "%CD%\start_gateway_lan.ps1" -ActuationSource "%ACTUATION_SOURCE%"

echo Waiting for gateway startup...
timeout /t 4 /nobreak >nul

echo.
echo [7/7] Starting Windows CARLA bridge in this window...
echo.
powershell -NoProfile -ExecutionPolicy Bypass -File "%CD%\start_carla_bridge.ps1" -ActuationSource "%ACTUATION_SOURCE%" -Scenario "%SCENARIO%"
set "RC=%ERRORLEVEL%"

echo.
echo CARLA bridge exited with code %RC%.
pause
exit /b %RC%

:fail
echo.
echo HIL launcher failed. Check the error above.
pause
exit /b 1
