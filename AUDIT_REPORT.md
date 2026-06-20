# AUDIT_REPORT

This report presents the findings of the repository validation pass for `uav-edge-llm-gateway`, targeting an Ubuntu 24.04 (ROS 2 Jazzy) environment.

## 🔴 Critical Findings

*   **[RESOLVED] SyntaxError in `llm_gateway_node.py` (Crash-causing)**
    *   **File:** `ros2_ws/src/drone_safety/scripts/llm_gateway_node.py`
    *   **Line:** 124
    *   **Observation:** The variable `latency_ms` is un-indented back to 12 spaces, prematurely terminating the `try` block (started at line 117). This causes a `SyntaxError: expected 'except' or 'finally' block` which entirely breaks compilation and will crash the node on startup.
    *   **Proposal:** Fix the indentation for the remainder of the `try` block, or move the `except` blocks to match the `try` statement's scope.
*   **[RESOLVED] SyntaxError in `benchmark_pipeline.py` (Crash-causing)**
    *   **File:** `scripts/benchmark_pipeline.py`
    *   **Line:** 272
    *   **Observation:** The variables `OLLAMA_URL` and `MODEL_NAME` are referenced locally at line 269 before their global declaration at line 272 (`global OLLAMA_URL, MODEL_NAME`). Python enforces that global declarations must precede any local usage, resulting in `SyntaxError: name 'OLLAMA_URL' is used prior to global declaration`.
    *   **Proposal:** Move the `global` declaration above the `self.declare_parameter` calls.

## 🟠 High Findings

*   **[RESOLVED] Regex strictly rejects valid nested JSON (Incorrect behavior)**
    *   **File:** `ros2_ws/src/drone_safety/scripts/llm_gateway_node.py` (Line 133) and `scripts/benchmark_pipeline.py` (Line 202)
    *   **Observation:** The regex `r'\{[^{}]*\}'` is designed to extract a JSON payload from the LLM output. However, it explicitly forbids inner braces (`{` or `}`). If the LLM produces extra keys containing nested JSON structures, the extraction fails completely, despite dictionary validation allowing extra keys.
    *   **Proposal:** Modify the regex pattern or use a stack-based bracket-matching algorithm to extract nested JSON blocks safely.
*   **[RESOLVED] Silent early return on uninitialized odometry (Untested safety path)**
    *   **File:** `ros2_ws/src/drone_safety/src/safety_filter.cpp`
    *   **Line:** 123-126
    *   **Observation:** If `has_odom` or `has_home` is false, `validate_and_update()` logs a warning and returns early. It does not update the internal state or explicitly engage a failsafe.
    *   **Proposal:** Instead of returning silently, explicitly call `engage_blind_halt()` or a similar failsafe before returning, to ensure the state machine is definitively safe.

## 🟡 Medium Findings

*   **[RESOLVED] Unpinned Python Dependencies (Reproducibility)**
    *   **File:** `requirements.txt`
    *   **Observation:** All dependencies use `>=` instead of `==` (e.g., `pandas>=2.0.0`). This leaves the environment open to breaking changes if major versions are released.
    *   **Proposal:** Pin dependencies to exact versions to guarantee reproducible builds.
*   **[RESOLVED] Orphaned Script (Dead Code)**
    *   **File:** `fix_crlf.py`
    *   **Observation:** This script exists in the repository root but is never referenced in `README.md`, `CMakeLists.txt`, or any launch file.
    *   **Proposal:** Delete the script if unused, or document its purpose in the `README.md`.

## ✅ Verified-Correct

*   **Coordinate Transformations (FLU to NED):** `safety_filter.cpp` (Lines 164-168). The 2D yaw rotation logic correctly transforms body-relative Forward-Left-Up coordinates to Earth-fixed North-East-Down frames. Sign conventions (e.g., Left mapping to West when facing North) align with the mathematical standards.
*   **Geofence & Kinematic Boundary Rejection:** `safety_filter.cpp` (Lines 177-192). The safety checks correctly throw explicit `std::runtime_error` exceptions (e.g., "Ceiling breached!") on out-of-bounds coordinates instead of silently clamping them. This properly triggers the `KINEMATIC_HOLD` failsafe.
*   **Dictionary/Key Validation:** `llm_gateway_node.py` (Line 136). The check `required.issubset(temp.keys())` properly ignores extra unexpected keys from the LLM while strictly enforcing the required keys (`action`, `x`, `y`, `z`).
*   **Cross-File CLI and Name Consistency:** All ROS 2 topics (`/llm/command_input`, `/llm/raw_output`), launch files (`full_system.launch.py`), and default parameters (`qwen2.5:3b`) match exactly between `README.md` and the source code.
*   **Runtime Smoke Test / Build Verification:** **`UNTESTABLE`**. The host environment is Windows, whereas the code inherently requires a Linux (Ubuntu 24.04) system with ROS 2 Jazzy and PX4 SITL/Gazebo installed to execute `colcon build` or headless node tests.
