#!/bin/bash
# setup.sh - Master setup script for Gaussian-LIC with submodules

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "🚀 Setting up Gaussian-LIC with submodules..."

# Initialize and update submodules
echo "📥 Initializing submodules..."
git submodule update --init --recursive

# Make chmod +x *.sh in setup directory
chmod +x ./setup/*.sh

# Install dependencies
echo "🔧 Installing dependencies..."
./setup/install_dependencies.sh

# Build workspace
echo "🔨 Building workspace..."
./setup/build_workspace.sh

# Setup environment
echo "⚙️ Setting up environment..."
./setup/setup_environment.sh

echo "✅ Setup completed successfully!"
echo ""
echo "🔧 Next steps:"
echo "   1. Source the environment: source ~/.bashrc"
echo "   2. Connect your sensors (ZED camera, Livox lidar)"
echo "   3. Launch: ros2 launch gaussian_lic your_launch_file.launch.py"