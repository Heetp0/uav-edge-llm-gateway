# UAV Edge LLM Gateway: Deploying Local Quantized Models

## 1. Project Title & Overview
The UAV Edge LLM Gateway is a fully localized, closed-loop AI system that translates natural language commands into real-time flight trajectories for Unmanned Aerial Vehicles (UAVs). Utilizing an onboard 4-bit quantized Large Language Model (Qwen-2.5-3B, Q4_K_M), the gateway eliminates cloud dependency and latency spikes. A deterministic C++ safety filter intercepts the LLM output, strictly enforcing kinematic limits and spatial geofences before dispatching setpoints to the flight controller.

## 2. System Architecture
The software pipeline bridges three fundamental components within an Ubuntu environment:
* **LLM Gateway Node (Python / Ollama):** An edge inference engine that parses unstructured natural language commands into structured JSON coordinate payloads.
* **Safety Filter Node (C++):** A deterministic tri-state machine (NOMINAL, KINEMATIC_HOLD, BLIND_HALT) that enforces a cylindrical geofence and mathematically validates all coordinates before conversion to MAVLink/uORB messages.
* **Benchmark Pipeline:** A Python-based automated testing suite that evaluates the LLM's spatial reasoning and latency across predefined command scenarios, logging metrics to CSV.

## 3. Prerequisites
Ensure the following standard software components are installed:
* Ubuntu 22.04 or 24.04 (native or WSL2)
* ROS 2 (Humble or Jazzy)
* PX4 Autopilot & Micro XRCE-DDS Agent
* Ollama (with the `qwen2.5:3b` model pulled locally)

## 4. Installation & Build

1. Clone the repository:
```bash
git clone https://github.com/Heetp0/uav-edge-llm-gateway.git
cd uav-edge-llm-gateway
```

2. Install the necessary Python dependencies:
```bash
pip install -r requirements.txt
```

3. Build the ROS 2 workspace:
```bash
cd ros2_ws
colcon build --symlink-install
```

4. Source the workspace:
```bash
source install/setup.bash
```

## 5. Configuration
System constraints and behavior are managed centrally in `config/geofence_params.yaml`. Modify this file to adjust the following safety limits:
* `max_altitude`: The maximum permitted altitude AGL (e.g., `100.0` meters).
* `max_range_xy`: The lateral radius of the cylindrical geofence anchored at the takeoff home position (e.g., `40.0` meters).
* `ground_noise_tolerance`: The margin for estimator noise at low altitudes to prevent false ground collision faults (e.g., `0.5` meters).

*Note: Changes to this file take effect on the next node startup. A workspace rebuild is not required.*

## 6. Running the System (SITL)
Execute the following commands in separate terminal instances to spin up the closed-loop stack.

1. Start the Ollama server:
```bash
ollama serve
```

2. Start the PX4 SITL environment (e.g., Gazebo):
```bash
cd ~/PX4-Autopilot
HEADLESS=1 make px4_sitl gz_x500_mono_cam
```

3. Start the Micro XRCE-DDS agent:
```bash
MicroXRCEAgent udp4 -p 8888
```

4. Launch the full ROS 2 system stack:
```bash
cd ~/uav-edge-llm-gateway/ros2_ws
source install/setup.bash
ros2 launch drone_safety full_system.launch.py
```
*(Optional args: `model:=qwen2.5:7b` or `log_level:=debug`)*

## 7. Benchmarking (Optional)
To evaluate the LLM's inference latency and semantic accuracy under hardware constraints, use the automated benchmark pipeline.

1. Run the benchmark pipeline launch file:
```bash
cd ~/uav-edge-llm-gateway/ros2_ws
source install/setup.bash
ros2 launch drone_safety benchmark.launch.py
```

2. Generate analytical plots from the resulting telemetry data:
```bash
cd ~/uav-edge-llm-gateway/data
python3 plot_results.py
```
