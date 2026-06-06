# UAV Edge LLM Gateway: Deploying Local Quantized Models

This repository contains the official implementation, ROS 2 workspace, and automated benchmarking suite for the research paper: 
**"UAV Edge LLM Gateway: Deploying Local Quantized Models for Autonomous Flight Control"**.

---

## 1. Overview
This project establishes a fully localized, closed-loop Edge AI gateway that translates natural language commands into real-time flight trajectories for Unmanned Aerial Vehicles (UAVs). 

By leveraging an onboard, 4-bit quantized Large Language Model (Qwen-2.5-3B, Q4_K_M), the system removes reliance on cloud-based APIs to eliminate latency spikes and internet dependency. To mitigate the "quantization tax" (semantic hallucinations and erratic syntactical outputs), a deterministic C++ Constraint Satisfaction Layer acts as a safety wrapper, intercepting and mathematically verifying all commands before they are dispatched to the flight stack.

---

## 2. System Architecture
The software pipeline runs natively inside **Ubuntu 24.04 LTS (via Windows Subsystem for Linux - WSL2)** and bridges three fundamental environments:
1. **Edge Inference Engine:** Ollama running the quantized `qwen2.5:3b` model to parse structural trajectory instructions into JSON strings.
2. **Deterministic Safety Filter:** A custom ROS 2 C++ node validating spatial boundaries, sign inversions, and battery capacities.
3. **Flight Control Stack:** PX4 Autopilot executing rigid-body kinematics in a headless Gazebo Software-in-the-Loop (SITL) environment linked via a Micro XRCE-DDS bridge.

---

## 3. Repository Structure
```text
uav-edge-llm-gateway/
├── ros2_ws/                  # ROS 2 Workspace
│   └── src/
│       └── drone_safety/     # Custom C++ safety filter & MAVLink/uORB mapper
├── scripts/                  # Automation Scripts
│   └── benchmark_pipeline.py # 20-command sequence evaluation script
├── data/                     # Metrics & Visualization
│   ├── benchmark_results.csv # Empirical timing & accuracy log
│   └── plot_results.py       # Latency distribution and jitter plotting script
├── .gitignore                # Exclusions for build/ and model weights
├── requirements.txt          # Python dependencies
└── README.md                 # Main documentation file
```

---

## 4. System Requirements & Installation

### Prerequisites
* Windows 11 with WSL2 (Ubuntu 24.04 LTS)
* ROS 2 (Jazzy or Humble)
* PX4 Autopilot Stack
* Ollama Core Engine

### Installation Steps

1. **Clone the Repository:**
   ```bash
   git clone [https://github.com/Heetp0/uav-edge-llm-gateway.git](https://github.com/Heetp0/uav-edge-llm-gateway.git)
   cd uav-edge-llm-gateway
   ```

2. **Pull the Local Quantized Model:**
   Ensure Ollama is running in the background, then pull the specific 4-bit model weight configuration:
   ```bash
   ollama pull qwen2.5:3b
   ```

3. **Install Python Dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

4. **Build the ROS 2 Workspace:**
   Navigate to the workspace folder, source your ROS 2 core environment, and compile your nodes:
   ```bash
   cd ros2_ws
   source /opt/ros/jazzy/setup.bash
   colcon build --symlink-install
   source install/setup.bash
   ```

---

## 5. Execution Guide

**CRITICAL NOTE: Every single step in this section MUST be executed in its own separate terminal instance (or tab). These are continuous background processes that must run concurrently to form the closed control loop.**

### Step 1: Launch Headless PX4 Autopilot SITL
*(Terminal 1)* Run the flight controller using the headless environment configuration flag to preserve maximum CPU/GPU processing bandwidth for the local inference engine:
```bash
cd ~/PX4-Autopilot
HEADLESS=1 make px4_sitl gz_x500_mono_cam
```

### Step 2: Initialize the Micro XRCE-DDS Middleware Bridge
*(Terminal 2)* Launch the communication agent over UDP port 8888 to translate internal uORB state topics to external ROS 2 publication schemas:
```bash
MicroXRCEAgent udp4 -p 8888
```

### Step 3: Run the Deterministic Safety Filter Node
*(Terminal 3)* Spin up the compiled C++ safety wrapper to begin listening for high-level JSON trajectories and evaluating spatial geofences:
```bash
cd ~/uav-edge-llm-gateway/ros2_ws
source install/setup.bash
ros2 run drone_safety safety_filter_node
```

### Step 4: Edge Hardware Simulation (CPU Throttling)
*(Terminal 4)* To accurately replicate the severe resource constraints of a UAV companion computer (e.g., Jetson Orin Nano) within a desktop WSL2 environment, the LLM inference engine must be artificially throttled **before** benchmarking.

1. **Install CPULimit:**
   ```bash
   sudo apt update && sudo apt install cpulimit
   ```

2. **Locate the Inference Engine PID:**
   Find the Process ID of the active Ollama background service:
   ```bash
   pgrep ollama
   ```

3. **Apply Compute Constraints:**
   Limit the localized process to a strict CPU percentage (e.g., 15% of a single core) using the PID retrieved above:
   ```bash
   sudo cpulimit -p <PID> -l 15 -b
   ```

### Step 5: Execute the Automated Benchmarking Suite
*(Terminal 5)* With the hardware constraints applied, run the evaluation script to feed the 20-command token pipeline sequentially to the LLM and capture edge execution metrics:
```bash
cd ~/uav-edge-llm-gateway/scripts
python3 benchmark_pipeline.py
```

---

## 6. Verification & Failsafe Logs
When an anomalous trajectory parameter is generated due to model weight compression, the safety node will capture the exception and print the following telemetry sequence to the terminal:

```text
[INFO] [safety_filter_node]: Ingesting asynchronous LLM JSON payload...
[ERROR] [safety_filter_node]: CRITICAL ANOMALY CAUGHT: Anomalous Vertical Vector Detected: Sign Inversion Suspected. Engaging Safe Auto-Hover Mode!
[WARN] [safety_filter_node]: MAVLink translation overridden. Safe-state published to uORB topic.
```

---

## 7. License
This project is licensed under the MIT License - see the LICENSE file for details.
