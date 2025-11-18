#!/bin/bash
# setup_environment.sh - Setup environment variables and aliases

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "⚙️ Setting up environment..."

# Create setup script
cat > "$SCRIPT_DIR/setup.bash" << 'EOF'
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
EOF

chmod +x "$SCRIPT_DIR/setup.bash"

# Add to bashrc if not already present
if ! grep -q "gaussian-lic-setup/setup.bash" ~/.bashrc; then
    echo "source $SCRIPT_DIR/setup.bash" >> ~/.bashrc
    echo "✅ Added workspace setup to ~/.bashrc"
fi

# Create useful aliases
cat > "$SCRIPT_DIR/.gaussian_aliases" << EOF
# Gaussian-LIC aliases
alias gaussian-build='cd $SCRIPT_DIR && colcon build --symlink-install'
alias gaussian-clean='cd $SCRIPT_DIR && rm -rf build install log'
alias gaussian-test='cd $SCRIPT_DIR && colcon test'
alias gaussian-rebuild='cd $SCRIPT_DIR && rm -rf build install && colcon build --symlink-install'

# Launch aliases (customize based on your launch files)
alias gaussian-run='ros2 launch gaussian_lic gs_mapping.launch.py'
alias livox-run='ros2 launch livox_ros2_driver livox_lidar_launch.py'

# Update aliases
alias gaussian-update='cd $SCRIPT_DIR && git pull && git submodule update --remote'
EOF

echo "source $SCRIPT_DIR/.gaussian_aliases" >> ~/.bashrc

echo "✅ Environment setup completed"
echo ""
echo "🔧 Useful commands:"
echo "   - gaussian-build: Build the workspace"
echo "   - gaussian-update: Update repository and submodules"
echo "   - gaussian-run: Launch Gaussian-LIC (customize launch command)"