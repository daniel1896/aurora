#!/bin/bash
# build_workspace.sh - Build the complete workspace

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
cd "$WORKSPACE_DIR"

echo "🔨 Building workspace..."

# Check if src directory exists and has submodules
if [ ! -d "src" ]; then
    echo "❌ src directory does not exist. Please run setup.sh first."
    exit 1
fi

if [ ! -d "src/gaussian_lic" ] || [ ! -d "src/livox_ros2_driver" ]; then
    echo "❌ Submodules missing in src directory."
    exit 1
fi

# Source ROS2
source /opt/ros/humble/setup.bash

# Install package dependencies
echo "📦 Installing package dependencies..."
rosdep install --from-paths src --ignore-src -r -y

# Build all packages
echo "🔨 Building all packages..."
colcon build --symlink-install --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CUDA_STANDARD=17

echo "✅ Workspace built successfully"