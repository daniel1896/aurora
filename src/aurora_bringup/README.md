# aurora_bringup

Unified bringup for **AURORA – DevDrone** (Pixhawk/ArduPilot + MAVROS, Livox Mid‑70, ZED 2i, FAST‑LIO2, TF, RViz).

## Build
```bash
# In your ROS 2 workspace
mkdir -p ~/aurora_ws/src && cd ~/aurora_ws/src
# Copy this folder into src/ then:
cd ~/aurora_ws
rosdep install --from-paths src --ignore-src -r -y || true
colcon build --symlink-install
source install/setup.bash
```

## Run
```bash
ros2 launch aurora_bringup bringup.launch.py \
    fcu_url:=/dev/ttyTHS1:921600 \
    livox_config:=$(ros2 pkg prefix aurora_bringup)/share/aurora_bringup/config/livox/mid70.json \
    fastlio_config:=$(ros2 pkg prefix aurora_bringup)/share/aurora_bringup/config/fast_lio/avia_mid70.yaml \
    zed_model:=zed2i zed_resolution:=HD720 zed_fps:=15 zed_depth_mode:=NEURAL \
    use_rviz:=true
```

### Notes
- If IMU topics are empty, set ArduPilot stream rates: `SR2_RAW_SENS=50`, `SR2_EXTRA1=50` (or `SR1_*` if the Jetson UART is TELEM1), then reboot.
- Adjust static TFs via launch args: `base_link → livox_frame` and `base_link → zed_camera_center`.
- By default we publish identity `odom→camera_init` to make RViz Fixed Frame **odom** work with FAST‑LIO2.
