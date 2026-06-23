param(
    [string]$NanoHost = "192.168.3.125",
    [string]$User = "jetson",
    [string]$RemoteDir = "/home/jetson/adas/hil"
)

$ErrorActionPreference = "Stop"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$gateway = Join-Path $here "hil_ros_gateway.py"

ssh "$User@$NanoHost" "mkdir -p '$RemoteDir'"
scp "$gateway" "$User@$NanoHost`:$RemoteDir/hil_ros_gateway.py"
ssh "$User@$NanoHost" "chmod +x '$RemoteDir/hil_ros_gateway.py' && ls -l '$RemoteDir/hil_ros_gateway.py'"
