# Ubuntu 24.04 HIL CARLA ROS2 package

This package runs the PC side of the HIL loop on Ubuntu 24.04 + ROS2 Jazzy.
CARLA ground-truth perception is published directly as ROS2 topics by
`pc/carla_ros_node.py`; the Jetson Nano controllers subscribe over DDS. Do not
start `nano/hil_ros_gateway.py` for this Ubuntu path.

## Runtime topology

```text
Ubuntu PC: CARLA + pc/carla_ros_node.py
  publishes:
    /car1_xy /car1_psi /car1_v
    /car2xy /car2_v /car2_class
    /road_psi /heng_error
  subscribes:
    /jetson/psi /jetson/delta /jetson/brake
    /esp32/psi /esp32/delta /esp32/brake
    /jetson/active_role /jetson/failover_available

Nano primary/backup: ADAS.py only, ROS_DOMAIN_ID=43
```

## Ubuntu setup

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=43
export ROS_LOCALHOST_ONLY=0

# Install CARLA Python client into the same Python used by ROS2 Jazzy.
pip install /path/to/CARLA/PythonAPI/carla/dist/carla-0.9.16-cp3xx-linux_x86_64.whl
python3 -c "import carla, rclpy; print('ok')"
```

Start CARLA separately:

```bash
/path/to/CARLA/CarlaUE4.sh -quality-level=Low
```

Then run the bridge:

```bash
cd hil_carla_ros2_ubuntu24
chmod +x launch/start_hil_ubuntu.sh
./launch/start_hil_ubuntu.sh acc jetson
```

Extra node arguments are passed through:

```bash
./launch/start_hil_ubuntu.sh acc jetson --carla-host 127.0.0.1 --town Town04 --duration 60
./launch/start_hil_ubuntu.sh acc esp32 --no-rendering
```

## Nano side

Run only the ADAS controller processes on ROS domain 43:

```bash
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=43
export ROS_LOCALHOST_ONLY=0
cd ~/adas/lx/SOCCode
python3 ADAS.py --role primary
```

Use `--role backup` on the backup Nano. The old TCP gateway is not part of this
Ubuntu direct-topic path.

## DDS check

On the Ubuntu PC:

```bash
export ROS_DOMAIN_ID=43
ros2 node list
ros2 topic list
ros2 topic hz /car1_xy
ros2 topic echo /jetson/delta
```

If the PC and Nano are not on the same LAN multicast domain, edit
`launch/fastdds_peers.xml` and set:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$PWD/launch/fastdds_peers.xml
```

If Jazzy and Foxy Fast DDS discovery is unstable, install CycloneDDS on both
ends and export:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```
