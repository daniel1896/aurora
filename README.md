# AURORA – DevDrone (Unified Bringup + Docker)

This repo skeleton contains:
- `src/aurora_bringup/launch/bringup.launch.py` — one-command bringup for MAVROS + Livox + ZED + FAST‑LIO2 + TF + RViz
- `docker/` — Jetson and x86 Dockerfiles and compose files with RViz working via X11

## Quick start (Jetson)
```bash
xhost +local:root
export L4T_TAG=r36.4.0    # match your JetPack
docker compose -f docker-compose.jetson.yml build
docker compose -f docker-compose.jetson.yml up
```
This will build the workspace at `/workspace/src` and open RViz.

## Quick start (x86_64)
```bash
xhost +local:root
docker compose -f docker-compose.x86.yml build
docker compose -f docker-compose.x86.yml up
```

## Launch the full stack (inside the container)
```bash
ros2 launch aurora_bringup bringup.launch.py \
  fcu_url:=/dev/ttyTHS1:921600 \
  zed_model:=zed2i zed_resolution:=HD720 zed_fps:=15 zed_depth_mode:=NEURAL \
  use_rviz:=true
```

### Notes
- Add your drivers/wrappers into `src/` as needed (e.g., `livox_ros2_driver`, `zed-ros2-wrapper`, `fast_lio`). The bringup includes them; clone/build if not installed system-wide.
- If IMU topics are empty: set `SR2_RAW_SENS=50`, `SR2_EXTRA1=50` (or `SR1_*` if TELEM1) in ArduPilot, then reboot.
- Edit `src/aurora_bringup/config/livox/mid70.json` to set your Livox `broadcast_code`.
