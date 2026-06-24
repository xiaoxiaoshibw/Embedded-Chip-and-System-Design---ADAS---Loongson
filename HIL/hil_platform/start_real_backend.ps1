param(
    [int]$Port = 8000,
    [string]$GatewayHost = "10.218.44.10",
    [string]$BackupHost = "10.218.44.155",
    [string]$PcHost = "10.218.44.190",
    [string]$CarlaHost = "127.0.0.1",
    [int]$CarlaPort = 2000,
    [string]$Town = "Town04"
)

$ErrorActionPreference = "Stop"

$env:HIL_MOCK = "0"
$env:HIL_CONTROL = "nano"
$env:GATEWAY_HOST = $GatewayHost
$env:BACKUP_HOST = $BackupHost
$env:PC_HOST = $PcHost
$env:NANO_USER = "jetson"
$env:NANO_PW_PRIMARY = "yahboom"
$env:NANO_PW_BACKUP = "jetson"
$env:NANO_FAULT_RESTORE_S = "8"
$env:PRIMARY_ADAS_CPUS = "0,1"
$env:PRIMARY_GATEWAY_CPUS = "2"
$env:BACKUP_ADAS_CPUS = "0,1"
$env:BACKUP_EDGE_CPUS = "2,3"
$env:CARLA_HOST = $CarlaHost
$env:CARLA_PORT = [string]$CarlaPort
$env:CARLA_TOWN = $Town
$env:CARLA_TM_PORT = "8010"
$env:HIL_CAMERA = "0"
$env:HIL_PORT = [string]$Port

python -m server.api_server
