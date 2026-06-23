$ErrorActionPreference = "Stop"

$killCmd = @"
python3 /home/jetson/adas/hil/stop_gateway.py || true
python3 - <<'PY'
import os, signal
for name in os.listdir('/proc'):
    if not name.isdigit():
        continue
    try:
        cmd=open('/proc/%s/cmdline'%name,'rb').read().replace(b'\0',b' ').decode('utf-8','replace')
    except Exception:
        continue
    if '/home/jetson/adas/lx/SOCCode/ADAS.py' in cmd:
        os.kill(int(name), signal.SIGTERM)
PY
"@

python (Join-Path $PSScriptRoot '..\tools\nano_ssh.py') B "$killCmd echo yahboom | sudo -S systemctl start adas-node.service"
python (Join-Path $PSScriptRoot '..\tools\nano_ssh.py') A "$killCmd echo jetson | sudo -S systemctl start adas-node.service"
