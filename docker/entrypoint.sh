#!/usr/bin/env bash
set -e
unset AMENT_TRACE_SETUP_FILES COLCON_TRACE
export AMENT_PYTHON_EXECUTABLE=${AMENT_PYTHON_EXECUTABLE:-$(command -v python3)}
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then source "/opt/ros/${ROS_DISTRO}/setup.bash"; fi
if [ -f "/workspace/install/setup.bash" ]; then source "/workspace/install/setup.bash"; fi
export QT_X11_NO_MITSHM=1
ls -l /dev/tty* 2>/dev/null | grep -E "ttyTHS|ttyUSB|ttyACM" || true
exec "$@"
