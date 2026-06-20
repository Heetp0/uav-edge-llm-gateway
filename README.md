# UAV Edge LLM Gateway

A deterministic safety architecture bridging natural language Large Language Models (LLMs) with hard real-time UAV flight controllers.

This repository allows an autonomous drone (via PX4 SITL) to process natural language spatial commands using locally hosted, heavily quantized AI models on constrained edge hardware, without compromising physical flight safety.

---

## 🚀 System Architecture

| Layer | Component | Role |
|---|---|---|
| Intelligence | Ollama (qwen2.5:3b) | Hosts the quantized LLM locally |
| Translation | `llm_gateway_node.py` | Subscribes to NL commands, queries LLM, publishes validated JSON |
| Safety | `safety_filter_node` (C++) | Enforces geofence, yaw transform, odometry watchdog, tri-state failsafe |
| Actuation | PX4 / XRCE-DDS | Executes sanitized trajectory setpoints in Gazebo |

**Topic graph:**
```
/llm/command_input → llm_gateway_node → /llm/raw_output → safety_filter_node → /fmu/in/trajectory_setpoint → PX4
```

---

## 🛠️ Prerequisites

Target environment: **Ubuntu 24.04** and **ROS 2 Jazzy**.

- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)
- [PX4 Autopilot](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html) (configured for SITL and Gazebo)
- [MicroXRCEAgent](https://micro-xrce-dds.docs.eprosima.com/en/latest/installation.html)
- [Ollama](https://ollama.com/download)

---

## ⚙️ Installation & Setup

**1. Clone the repository**
```bash
git clone https://github.com/Heetp0/uav-edge-llm-gateway.git
cd uav-edge-llm-gateway
```

**2. Run the Ubuntu Setup Script**
This script automates the installation of ROS 2 Jazzy, system dependencies (`colcon`, `nlohmann-json3-dev`, `cpulimit`), Python requirements, and clones the missing `px4_msgs` repository.
```bash
chmod +x setup_ubuntu.sh
./setup_ubuntu.sh
```

**3. Install Ollama**
Install the Ollama engine, which handles the local LLM inference.
```bash
curl -fsSL https://ollama.com/install.sh | sh
```

**4. Pull the edge-optimized AI model**
```bash
ollama pull qwen2.5:3b
```

**5. Build the ROS 2 workspace**
```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
cd ..
```

---

## ⚙️ Configuration

System constraints and behaviour are managed centrally in `config/geofence_params.yaml`. Modify this file to adjust safety limits without rebuilding:

| Parameter | Default | Description |
|---|---|---|
| `max_altitude` | `100.0` m | Maximum permitted altitude AGL |
| `max_range_xy` | `40.0` m | Lateral radius of the cylindrical geofence from home position |
| `ground_noise_tolerance` | `0.5` m | Estimator noise margin at low altitude to prevent false ground collision faults |
| `watchdog_timeout_sec` | `0.5` s | Maximum age of odometry before forcing BLIND_HALT |

> Changes take effect on the next node startup. A workspace rebuild is not required.

---

## 🚁 Launch Files

The system has three launch files covering distinct stages of development and operation.

### `full_system.launch.py` — One command, full stack

Starts everything in the correct order automatically:

```
t=0s   ollama serve
t=0s   PX4 SITL + Gazebo
t=0s   MicroXRCEAgent (DDS bridge)
t=5s   safety_filter_node
t=7s   llm_gateway_node
```

```bash
source /opt/ros/jazzy/setup.bash
source ~/uav-edge-llm-gateway/ros2_ws/install/setup.bash
ros2 launch drone_safety full_system.launch.py
```

Optional arguments:
```bash
ros2 launch drone_safety full_system.launch.py model:=qwen2.5:7b
ros2 launch drone_safety full_system.launch.py log_level:=debug
ros2 launch drone_safety full_system.launch.py headless:=false
```

Once running, send commands from any terminal:
```bash
ros2 topic pub --once /llm/command_input std_msgs/String "data: 'Fly forward 5 meters'"
```

Monitor the pipeline:
```bash
ros2 topic echo /llm/raw_output    # validated JSON from LLM
ros2 topic echo /llm/status        # inference status per command
```

---

### `safety_filter.launch.py` — Filter in isolation

Starts only the C++ safety filter node. Use this during early SITL debugging to test the geofence, coordinate transform, and tri-state machine without involving Ollama.

```bash
ros2 launch drone_safety safety_filter.launch.py
ros2 launch drone_safety safety_filter.launch.py log_level:=debug
```

Feed it commands manually to verify behaviour:
```bash
# Valid command — should reach PX4
ros2 topic pub --once /llm/raw_output std_msgs/String \
  "data: '{\"action\":\"goto\",\"x\":5.0,\"y\":0.0,\"z\":0.0}'"

# Geofence breach — should trigger KINEMATIC_HOLD
ros2 topic pub --once /llm/raw_output std_msgs/String \
  "data: '{\"action\":\"goto\",\"x\":100.0,\"y\":0.0,\"z\":0.0}'"
```

---

### `benchmark.launch.py` — Accuracy and latency benchmark

Starts the safety filter and benchmark pipeline. Does **not** start PX4, the DDS bridge, or Ollama — those must be running separately. The benchmark publishes real setpoints but without PX4 connected nothing acts on them, which is intentional.

```bash
# Terminal 1 — start Ollama first
ollama serve

# Optional: simulate edge hardware constraints
sudo cpulimit -p $(pgrep -n ollama) -l 400 -b

# Terminal 2 — run the benchmark
source /opt/ros/jazzy/setup.bash
source ~/uav-edge-llm-gateway/ros2_ws/install/setup.bash
ros2 launch drone_safety benchmark.launch.py
```

Optional arguments:
```bash
ros2 launch drone_safety benchmark.launch.py model:=qwen2.5:7b
ros2 launch drone_safety benchmark.launch.py log_level:=debug
```

Results are saved to `~/quantization_benchmark_results.csv` automatically. The benchmark is resume-capable — if interrupted, restarting continues from the last completed row.

Generate performance charts after the benchmark completes:
```bash
ros2 run drone_safety plot_results

# Or with a custom output directory
python3 data/plot_results.py --out ~/Desktop/plots
```

Output charts:
- `latency_distribution.png` — inference latency box plot
- `accuracy_metrics.png` — syntactic and semantic success rates
- `latency_time_series.png` — latency over sequential execution order
- `per_command_accuracy.png` — per-command semantic accuracy breakdown

---

## 📋 Launch File Summary

| Launch file | Ollama | PX4 + Gazebo | DDS bridge | Safety filter | Gateway | Benchmark |
|---|---|---|---|---|---|---|
| `full_system.launch.py` | ✅ auto | ✅ auto | ✅ auto | ✅ | ✅ | ❌ |
| `safety_filter.launch.py` | ❌ | ❌ | ❌ | ✅ | ❌ | ❌ |
| `benchmark.launch.py` | ❌ manual | ❌ | ❌ | ✅ | ❌ | ✅ |

---

## 🔧 Troubleshooting

### Mixed OS Environments (Windows / Linux)
If you clone this repository on Windows and attempt to run it on Linux (or via WSL), scripts might fail to execute due to `\r\n` (CRLF) line endings. You can resolve this using the included utility script from the repository root:

```bash
python3 fix_crlf.py
```
This script recursively scans the project and normalizes all `.py`, `.sh`, `.yaml`, `.xml`, `.txt`, and `.md` files to use Unix `\n` (LF) line endings.
