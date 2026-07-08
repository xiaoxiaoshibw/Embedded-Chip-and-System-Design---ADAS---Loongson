param(
    [int]$RosDomainId = 43
)

$ErrorActionPreference = "Stop"

$runtimeEnv = "ADAS_CPU_LIST='0,1,2' RT_CONTROL_CORE=0 RT_AUX_CORES=1 LOCKSTEP_ENABLED=1 LOCKSTEP_CHECKER_CORE=2"

python (Join-Path $PSScriptRoot '..\tools\nano_ssh.py') B "python3 /home/jetson/adas/hil/stop_gateway.py; $runtimeEnv python3 /home/jetson/adas/hil/start_hil_adas.py --role primary --domain $RosDomainId --sudo-password yahboom"
python (Join-Path $PSScriptRoot '..\tools\nano_ssh.py') A "python3 /home/jetson/adas/hil/stop_gateway.py; $runtimeEnv python3 /home/jetson/adas/hil/start_hil_adas.py --role backup --domain $RosDomainId --sudo-password jetson"
