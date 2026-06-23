param(
    [ValidateSet("B", "A", "both")]
    [string]$Target = "both",
    [string]$RemoteDir = "/home/jetson/adas/hil"
)

$ErrorActionPreference = "Stop"
# launch/ → carla_bridge/；只上传 nano/ 部署单元（扁平到 Nano /home/jetson/adas/hil）
$bridge = Split-Path -Parent $PSScriptRoot
$nanoDir = Join-Path $bridge "nano"
$upload = Join-Path $bridge "tools\upload.py"
$nanoSsh = Join-Path $bridge "tools\nano_ssh.py"

Write-Host "Uploading carla_bridge/nano/ to $Target`:$RemoteDir ..."
if ($Target -eq "both") {
    python $upload B $nanoDir $RemoteDir
    python $upload A $nanoDir $RemoteDir
} else {
    python $upload $Target $nanoDir $RemoteDir
}

Write-Host "Remote verification ..."
python $nanoSsh $Target "source /opt/ros/foxy/setup.bash; python3 -m py_compile '$RemoteDir/hil_ros_gateway.py' '$RemoteDir/start_hil_adas.py' '$RemoteDir/stop_gateway.py'; ls -l '$RemoteDir'"
