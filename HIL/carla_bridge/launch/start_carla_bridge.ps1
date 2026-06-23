param(
    [ValidateSet("jetson", "esp32")]
    [string]$ActuationSource = "jetson",
    [string]$Scenario = "acc",
    [string]$GatewayHost = "192.168.3.125",
    [int]$SensorPort = 42100,
    [int]$ActuationPort = 42101,
    [int]$TcpPort = 42110,
    [switch]$NoRendering
)

$ErrorActionPreference = "Stop"
$argsList = @(
    (Join-Path $PSScriptRoot '..\pc\hil_carla_bridge.py'),
    "--scenario", $Scenario,
    "--gateway-host", $GatewayHost,
    "--sensor-port", $SensorPort,
    "--actuation-port", $ActuationPort,
    "--tcp-port", $TcpPort,
    "--actuation-source", $ActuationSource
)
if ($NoRendering) {
    $argsList += "--no-rendering"
}

python @argsList
