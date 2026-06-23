param(
    [ValidateSet("jetson", "esp32")]
    [string]$ActuationSource = "jetson",
    [string]$NanoHost = "192.168.3.125",
    [string]$PcHost = "192.168.3.8",
    [string]$User = "jetson",
    [int]$SensorPort = 42100,
    [int]$ActuationPort = 42101
)

$ErrorActionPreference = "Stop"
$remote = "/home/jetson/adas/hil/hil_ros_gateway.py"
$cmd = @"
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
python3 '$remote' --pc-host '$PcHost' --sensor-port $SensorPort --actuation-port $ActuationPort --actuation-source '$ActuationSource'
"@

ssh "$User@$NanoHost" $cmd
