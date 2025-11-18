#!/bin/bash
# build_workspace.sh - Build the complete workspace

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "🔨 Building workspace..."

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