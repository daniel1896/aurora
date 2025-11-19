#!/bin/bash
# Environment setup for Gaussian-LIC workspace

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source ROS2
source /opt/ros/humble/setup.bash

# Source workspace
if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    source "$SCRIPT_DIR/install/setup.bash"
fi

export GAUSSIAN_LIC_WS="$SCRIPT_DIR"

echo "🚀 Environment ready for Gaussian-LIC + Livox + MAVROS"
echo "   Workspace: $GAUSSIAN_LIC_WS"
