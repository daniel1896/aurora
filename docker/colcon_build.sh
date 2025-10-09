# Replace the header with this (or apply the sed lines below)
#!/usr/bin/env bash
set -eo pipefail

# Silence optional tracing (these being *set* triggers verbose output)
unset AMENT_TRACE_SETUP_FILES COLCON_TRACE
export AMENT_PYTHON_EXECUTABLE=${AMENT_PYTHON_EXECUTABLE:-$(command -v python3)}

WS=${1:-/workspace}
echo ">>> ROS 2 build in: ${WS}"

# Source ROS
source /opt/ros/${ROS_DISTRO}/setup.bash

# rosdep is optional – run if present
if command -v rosdep >/dev/null 2>&1; then
  rosdep update || true
  rosdep install --from-paths ${WS}/src --ignore-src -r -y || true
fi

cd ${WS}
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Source the workspace *but don't die if it returns non-zero*
[ -f "${WS}/install/setup.bash" ] && source "${WS}/install/setup.bash" || true

echo ">>> Build complete"
