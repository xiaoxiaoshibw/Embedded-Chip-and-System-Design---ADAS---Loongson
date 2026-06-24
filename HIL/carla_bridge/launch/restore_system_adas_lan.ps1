$ErrorActionPreference = "Stop"

# 收尾：先停受监管的 HIL transient 单元（adas-hil-<role>.service）——必须先停单元，
# 否则 Restart=always 会把随后 pkill 掉的 ADAS 立刻拉回来，根本停不掉。再清掉任何裸
# 孤儿 ADAS，最后恢复生产 adas-node.service。
$orphanKill = @"
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

$nanoSsh = Join-Path $PSScriptRoot '..\tools\nano_ssh.py'

# B (primary, sudo=yahboom)
python $nanoSsh B "echo yahboom | sudo -S systemctl stop adas-hil-primary.service adas-hil-backup.service 2>/dev/null || true; $orphanKill echo yahboom | sudo -S systemctl reset-failed adas-node.service 2>/dev/null || true; echo yahboom | sudo -S systemctl start adas-node.service"
# A (backup, sudo=jetson)
python $nanoSsh A "echo jetson | sudo -S systemctl stop adas-hil-primary.service adas-hil-backup.service 2>/dev/null || true; $orphanKill echo jetson | sudo -S systemctl reset-failed adas-node.service 2>/dev/null || true; echo jetson | sudo -S systemctl start adas-node.service"
