param(
    [string]$PrimaryHost = "192.168.3.125",
    [string]$BackupHost = "192.168.3.124",
    [string]$PrimaryUser = "jetson",
    [string]$BackupUser = "jetson"
)

$ErrorActionPreference = "Stop"

ssh "$PrimaryUser@$PrimaryHost" "pkill -f perception_sim.py || true; pkill -f perception_sim || true; source /opt/ros/foxy/setup.bash; export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0; timeout 3 ros2 node list || true"
ssh "$BackupUser@$BackupHost" "pkill -f perception_sim.py || true; pkill -f perception_sim || true; source /opt/ros/foxy/setup.bash; export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0; timeout 3 ros2 node list || true"
