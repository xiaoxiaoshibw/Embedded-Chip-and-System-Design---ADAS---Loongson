param(
    [int]$RosDomainId = 43
)

$ErrorActionPreference = "Stop"

python (Join-Path $PSScriptRoot '..\tools\nano_ssh.py') B "python3 /home/jetson/adas/hil/stop_gateway.py; python3 /home/jetson/adas/hil/start_hil_adas.py --role primary --domain $RosDomainId --sudo-password yahboom"
python (Join-Path $PSScriptRoot '..\tools\nano_ssh.py') A "python3 /home/jetson/adas/hil/stop_gateway.py; python3 /home/jetson/adas/hil/start_hil_adas.py --role backup --domain $RosDomainId --sudo-password jetson"
