@echo off
setlocal EnableExtensions
chcp 65001 >nul

cd /d "%~dp0"

set "BACKEND_PORT=%~1"
if "%BACKEND_PORT%"=="" set "BACKEND_PORT=8000"

set "WEB_PORT=%~2"
if "%WEB_PORT%"=="" set "WEB_PORT=5173"

set "GATEWAY_HOST=10.218.44.10"
set "BACKUP_HOST=10.218.44.155"
set "PC_HOST=10.218.44.190"
set "PRIMARY_ADAS_CPUS=0,1"
set "PRIMARY_GATEWAY_CPUS=2"
set "BACKUP_ADAS_CPUS=0,1"
set "BACKUP_EDGE_CPUS=2,3"

echo ============================================================
echo  ADAS HIL WebUI launcher
echo  Backend: http://127.0.0.1:%BACKEND_PORT%
echo  WebUI:   http://127.0.0.1:%WEB_PORT%/live
echo  Primary Nano ZeroTier: %GATEWAY_HOST%
echo  Backup  Nano ZeroTier: %BACKUP_HOST%
echo  PC      ZeroTier:      %PC_HOST%
echo  CPU map: 125 ADAS=%PRIMARY_ADAS_CPUS%, 125 Gateway=%PRIMARY_GATEWAY_CPUS%, 124 ADAS=%BACKUP_ADAS_CPUS%, 124 Edge=%BACKUP_EDGE_CPUS%
echo ============================================================
echo.

start "HIL Real Backend" /D "%CD%" powershell -NoExit -NoProfile -ExecutionPolicy Bypass -File "%CD%\start_real_backend.ps1" -Port %BACKEND_PORT% -GatewayHost "%GATEWAY_HOST%" -BackupHost "%BACKUP_HOST%" -PcHost "%PC_HOST%"

echo Waiting for backend startup...
timeout /t 3 /nobreak >nul

start "HIL WebUI" /D "%CD%\web" cmd /k "set HIL_API=http://127.0.0.1:%BACKEND_PORT%&& npm run dev -- --host 127.0.0.1 --port %WEB_PORT%"

echo.
echo Open http://127.0.0.1:%WEB_PORT%/live
echo Use the hardware panel to run: one-click prepare HIL, load scenario, start simulation.
echo.
pause
