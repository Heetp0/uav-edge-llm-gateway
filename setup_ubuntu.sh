#!/bin/bash
set -e

echo "=== UAV Edge LLM Gateway: Ubuntu 24.04 Setup ==="

# 1. Setup ROS 2 repositories
echo "[1/6] Setting up ROS 2 Jazzy repositories..."
sudo apt update && sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common curl -y
sudo add-apt-repository universe -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 2. Install apt dependencies
echo "[2/6] Installing apt dependencies (ROS 2, Colcon, JSON, CPULimit)..."
sudo apt update
sudo apt install ros-jazzy-desktop python3-colcon-common-extensions nlohmann-json3-dev cpulimit zstd pciutils cmake build-essential git -y

# 3. Clone px4_msgs
echo "[3/6] Cloning px4_msgs into ros2_ws/src..."
mkdir -p ros2_ws/src
if [ ! -d "ros2_ws/src/px4_msgs" ]; then
    git clone https://github.com/PX4/px4_msgs.git ros2_ws/src/px4_msgs
else
    echo "px4_msgs already exists. Skipping clone."
fi

# 4. Install Micro-XRCE-DDS-Agent
echo "[4/6] Installing Micro-XRCE-DDS-Agent..."
if [ ! -d "$HOME/Micro-XRCE-DDS-Agent" ]; then
    # Save current directory to return to it later
    ORIGINAL_DIR=$(pwd)
    cd "$HOME"
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
    cd Micro-XRCE-DDS-Agent
    mkdir -p build && cd build
    cmake ..
    make
    sudo make install
    sudo ldconfig /usr/local/lib/
    cd "$ORIGINAL_DIR"
else
    echo "Micro-XRCE-DDS-Agent already exists. Skipping installation."
fi

# 5. Clone PX4-Autopilot
echo "[5/6] Cloning and configuring PX4-Autopilot..."
if [ ! -d "$HOME/PX4-Autopilot" ]; then
    ORIGINAL_DIR=$(pwd)
    cd "$HOME"
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive
    bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
    cd "$ORIGINAL_DIR"
else
    echo "PX4-Autopilot already exists. Skipping clone."
fi

# 6. Install python dependencies
echo "[6/6] Installing Python dependencies..."
# Use --break-system-packages for Ubuntu 24.04 PEP-668 compliance if run outside a venv
pip install -r requirements.txt --break-system-packages || pip install -r requirements.txt

echo "=== Setup Complete! ==="
echo "NOTE: If this is your first time running the PX4 setup script, you MUST restart your computer or log out/log back in before continuing!"
echo "You can now run 'ollama pull qwen2.5:3b' and build the workspace with 'colcon build'."
