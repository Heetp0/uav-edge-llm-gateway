# UAV Edge LLM Gateway
A deterministic safety architecture bridging natural language Large Language Models (LLMs) with hard real-time UAV flight controllers.
This repository allows an autonomous drone (via PX4 SITL) to process natural language spatial commands using locally hosted, heavily quantized AI models on constrained edge hardware, without compromising physical flight safety.
## 🚀 System Architecture
 * **Intelligence Layer (Ollama):** Hosts the quantized qwen2.5:3b LLM locally.
 * **Translation Layer (Python):** Subscribes to natural language inputs, prompts the LLM, and formats the output into strict JSON 3D coordinates.
 * **Safety Gateway (C++):** Intercepts the AI's JSON. Acts as a deterministic hard-stop, enforcing cylindrical geofences and kinematic laws while rejecting hallucinations.
 * **Actuation Layer (PX4 / DDS):** Executes the sanitized MAVLink setpoints in the Gazebo physics engine.
## 🛠️ Prerequisites
Target environment: **Ubuntu 24.04** and **ROS 2 Jazzy**.
 * ROS 2 Jazzy
 * PX4 Autopilot (configured for SITL & Gazebo)
 * MicroXRCEAgent
 * Ollama
## ⚙️ Installation & Setup
**1. Clone the repository**
```bash
git clone [https://github.com/Heetp0/uav-edge-llm-gateway.git](https://github.com/Heetp0/uav-edge-llm-gateway.git)
cd uav-edge-llm-gateway

```
**2. Install Python dependencies**
```bash
pip install -r requirements.txt

```
**3. Download the Edge-Optimized AI Model**
```bash
ollama pull qwen2.5:3b-instruct-q4_0

```
**4. Build the ROS 2 Workspace**
```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
cd ..

```
## 🚁 Running the Full Flight Stack
Split your terminal into 5 separate panes to monitor the data flow safely.
**Pane 1: Boot PX4 Physics (Gazebo)**
```bash
cd ~/PX4-Autopilot
HEADLESS=1 make px4_sitl gz_x500_mono_cam

```
**Pane 2: Boot the DDS Network Bridge**
```bash
MicroXRCEAgent udp4 -p 8888

```
**Pane 3: Boot the C++ Safety Filter**
```bash
source /opt/ros/jazzy/setup.bash
source ~/uav-edge-llm-gateway/ros2_ws/install/setup.bash
ros2 run drone_safety safety_filter_node

```
**Pane 4: Apply Edge Hardware Constraints**
```bash
sudo systemctl restart ollama
sudo cpulimit -p $(pgrep ollama) -l 400 -b

```
**Pane 5: Launch the AI Translation Gateway**
*(Note: Executed directly via Python to bypass current CMake install configuration)*
```bash
source /opt/ros/jazzy/setup.bash
source ~/uav-edge-llm-gateway/ros2_ws/install/setup.bash
python3 ~/uav-edge-llm-gateway/ros2_ws/src/drone_safety/scripts/llm_gateway_node.py

```
## 📊 Running the Automated Benchmark Suite
The benchmarking pipeline evaluates latency, syntactic integrity, and semantic spatial reasoning.
**1. Run the Evaluation Pipeline:**
Instead of running the gateway node in Pane 5, execute the benchmark script from the root repository scripts directory:
```bash
source /opt/ros/jazzy/setup.bash
cd ~/uav-edge-llm-gateway/scripts
python3 benchmark_pipeline.py

```
**2. Generate Performance Graphs:**
Plot the hardware metrics using the script located in the root repository data directory:
```bash
cd ~/uav-edge-llm-gateway/data
python3 plot_results.py

```
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
