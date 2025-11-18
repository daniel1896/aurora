#!/bin/bash
# install_dependencies.sh - Install all required dependencies

set -e

echo "🔧 Installing all dependencies..."

# Install ROS2 Humble if not present
if ! command -v ros2 &> /dev/null; then
    echo "📦 Installing ROS2 Humble..."
    sudo apt update
    sudo apt install -y software-properties-common curl gnupg lsb-release
    
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    sudo apt update
    sudo apt install -y ros-humble-desktop-full python3-rosdep python3-colcon-common-extensions
    
    sudo rosdep init || true
    rosdep update
fi

# Source ROS2
source /opt/ros/humble/setup.bash

# Install system dependencies
echo "📦 Installing system dependencies..."
sudo apt install -y \
    git cmake build-essential \
    libopencv-dev libpcl-dev libeigen3-dev \
    libyaml-cpp-dev libffi-dev \
    libglew-dev libglfw3-dev libx11-dev libglm-dev \
    pkg-config python3-pip \
    openmpi-bin libopenmpi-dev

# Install MAVROS
echo "📦 Installing MAVROS..."
sudo apt install -y \
    ros-humble-mavros \
    ros-humble-mavros-msgs \
    ros-humble-mavros-extras \
    ros-humble-mavlink

sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

# Install ROS dependencies
echo "📦 Installing ROS dependencies..."
sudo apt install -y \
    ros-humble-cv-bridge \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-image-transport \
    ros-humble-tf2-ros \
    ros-humble-tf2-eigen \
    ros-humble-nav-msgs

# Install Python dependencies
echo "📦 Installing Python dependencies..."
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip3 install numpy opencv-python pyyaml

# Install Pangolin
echo "📦 Installing Pangolin..."
if [ ! -f /usr/local/include/pangolin/pangolin.h ] && [ ! -f /usr/include/pangolin/pangolin.h ]; then
    BUILD_DIR="/tmp/pangolin_build"
    mkdir -p "$BUILD_DIR"
    cd "$BUILD_DIR"
    git clone --depth 1 -b v0.8 https://github.com/stevenlovegrove/Pangolin.git
    cd Pangolin
    mkdir build && cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release
    make -j$(nproc)
    sudo make install
    rm -rf "$BUILD_DIR"
fi

echo "✅ All dependencies installed successfully"