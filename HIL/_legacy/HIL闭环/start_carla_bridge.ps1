param(
    [ValidateSet("jetson", "esp32")]
    [string]$ActuationSource = "jetson",
    [string]$Scenario = "acc",
    [string]$GatewayHost = "192.168.3.125",
    [int]$SensorPort = 42100,
    [int]$ActuationPort = 42101,
    [switch]$NoRendering
)

$ErrorActionPreference = "Stop"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$argsList = @(
    (Join-Path $here "hil_carla_bridge.py"),
    "--scenario", $Scenario,
    "--gateway-host", $GatewayHost,
    "--sensor-port", $SensorPort,
    "--actuation-port", $ActuationPort,
    "--actuation-source", $ActuationSource
)
if ($NoRendering) {
    $argsList += "--no-rendering"
}

python @argsList
