param(
    [ValidateSet("jetson", "esp32")]
    [string]$ActuationSource = "jetson",
    [string]$PcHost = "192.168.3.8",
    [int]$SensorPort = 42100,
    [int]$ActuationPort = 42101,
    [int]$TcpPort = 42110,
    [int]$RosDomainId = 43,
    [string]$RemoteDir = "/home/jetson/adas/hil"
)

$ErrorActionPreference = "Stop"
$remote = "$RemoteDir/hil_ros_gateway.py"
$stop = "$RemoteDir/stop_gateway.py"
$cmd = "python3 '$stop' || true; source /opt/ros/foxy/setup.bash; export ROS_DOMAIN_ID=$RosDomainId ROS_LOCALHOST_ONLY=0; python3 '$remote' --pc-host '$PcHost' --sensor-port $SensorPort --actuation-port $ActuationPort --tcp-port $TcpPort --actuation-source '$ActuationSource'"

python (Join-Path $PSScriptRoot '..\tools\nano_ssh.py') B $cmd
