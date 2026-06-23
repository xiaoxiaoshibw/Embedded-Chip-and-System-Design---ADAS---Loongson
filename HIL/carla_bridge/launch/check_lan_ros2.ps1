param(
    [string]$PrimaryHost = "192.168.3.125",
    [string]$BackupHost = "192.168.3.124",
    [int]$RosDomainId = 43
)

$ErrorActionPreference = "Stop"

Write-Host "== LAN ping =="
ping -n 2 $PrimaryHost
ping -n 2 $BackupHost

Write-Host "== SSH + ROS2 graph =="
python (Join-Path $PSScriptRoot '..\tools\nano_ssh.py') both "source /opt/ros/foxy/setup.bash; export ROS_DOMAIN_ID=$RosDomainId ROS_LOCALHOST_ONLY=0; echo HOST=\$(hostname); timeout 6 ros2 node list | sort; echo ---topics---; timeout 6 ros2 topic list | sort | grep -E '^/(adas|car|esp32|heng|jetson|road)'"
