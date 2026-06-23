# LAN HIL bridge

This folder contains the real hardware-in-the-loop bridge for:

```text
CARLA on Windows -> LAN UDP -> Nano ROS2 gateway -> existing ADAS.py
existing ADAS.py -> /jetson/* or /esp32/* -> gateway -> LAN UDP -> CARLA
```

Windows does not need ROS2 and does not run the ADAS control stack. ROS2 stays
on the Jetson Nano boards.

## LAN addresses

- Windows/CARLA PC: `192.168.3.8`
- Primary Nano B: `192.168.3.125`, user `jetson`, password `yahboom`
- Backup Nano A: `192.168.3.124`, user `jetson`, password `jetson`
- ROS domain: `42`

## Files

- `hil_carla_bridge.py`: Windows-side CARLA bridge. Publishes CARLA truth frames
  to the Nano gateway and applies returned actuation to the ego vehicle.
- `hil_ros_gateway.py`: Nano-side ROS2 gateway. Publishes CARLA truth to the
  existing ADAS input topics and returns `/jetson/*` or `/esp32/*` output.
- `deploy_gateway.ps1`: Uploads `hil_ros_gateway.py` to the primary Nano.
- `start_gateway_lan.ps1`: Starts the gateway on the primary Nano.
- `stop_perception_sim_lan.ps1`: Stops the old `/perception_sim` publisher.
- `start_carla_bridge.ps1`: Starts the Windows CARLA bridge.

## Recommended startup

1. Stop the old simulated perception publisher:

   ```powershell
   .\stop_perception_sim_lan.ps1
   ```

2. Upload the gateway:

   ```powershell
   .\deploy_gateway.ps1
   ```

3. Start the gateway on `192.168.3.125`:

   ```powershell
   .\start_gateway_lan.ps1 -ActuationSource jetson
   ```

4. Start `CALRA\CarlaUE4.exe`.

5. Run the Windows bridge:

   ```powershell
   .\start_carla_bridge.ps1 -ActuationSource jetson -Scenario acc
   ```

After the ESP32 serial readback no longer reports `persistent parse error`,
switch both gateway and bridge to the final full-HIL path:

```powershell
.\start_gateway_lan.ps1 -ActuationSource esp32
.\start_carla_bridge.ps1 -ActuationSource esp32 -Scenario acc
```

## Validation commands

```powershell
python ..\lx\_nano_deploy\nano_ssh.py both "source /opt/ros/foxy/setup.bash; export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0; ros2 node list; ros2 topic list"
```

The ROS2 graph should contain `/adas_primary`, `/adas_backup`,
`/car1_xy`, `/car1_psi`, `/car1_v`, `/car2xy`, `/car2_v`, `/road_psi`,
`/heng_error`, `/jetson/*`, and `/esp32/*`.
