#!/bin/bash
set -e

echo "=== UAV Edge LLM Gateway: Ubuntu 24.04 Setup ==="

# 1. Setup ROS 2 repositories
echo "[1/4] Setting up ROS 2 Jazzy repositories..."
sudo apt update && sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common curl -y
sudo add-apt-repository universe -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 2. Install apt dependencies
echo "[2/4] Installing apt dependencies (ROS 2, Colcon, JSON, CPULimit)..."
sudo apt update
sudo apt install ros-jazzy-desktop python3-colcon-common-extensions nlohmann-json3-dev cpulimit zstd pciutils -y

# 3. Clone px4_msgs
echo "[3/4] Cloning px4_msgs into ros2_ws/src..."
mkdir -p ros2_ws/src
if [ ! -d "ros2_ws/src/px4_msgs" ]; then
    git clone https://github.com/PX4/px4_msgs.git ros2_ws/src/px4_msgs
else
    echo "px4_msgs already exists. Skipping clone."
fi

# 4. Install python dependencies
echo "[4/4] Installing Python dependencies..."
# Use --break-system-packages for Ubuntu 24.04 PEP-668 compliance if run outside a venv
pip install -r requirements.txt --break-system-packages || pip install -r requirements.txt

echo "=== Setup Complete! ==="
echo "You can now run 'ollama pull qwen2.5:3b' and build the workspace with 'colcon build'."
