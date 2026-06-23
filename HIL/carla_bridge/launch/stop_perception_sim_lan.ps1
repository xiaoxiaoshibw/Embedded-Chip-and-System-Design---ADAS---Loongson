$ErrorActionPreference = "Stop"

python (Join-Path $PSScriptRoot '..\tools\nano_ssh.py') both "pkill -f perception_sim.py || true; pkill -f perception_sim || true; source /opt/ros/foxy/setup.bash; export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0; timeout 4 ros2 node list | sort"
