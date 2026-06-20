#!/usr/bin/env python3

import os
import re
import csv
import json
import time
import math
import concurrent.futures
from collections import deque

import requests

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

OLLAMA_URL = "http://localhost:11434/api/generate"
MODEL_NAME = "qwen2.5:3b"
TOTAL_RUNS = 20
MAX_RETRIES = 3
BACKOFF_BASE = 3.0
CONNECT_TIMEOUT = 5
READ_TIMEOUT = 90
SESSION_RECYCLE = 100
LATENCY_WINDOW = 10
MIN_SLEEP = 0.5
MAX_SLEEP = 6.0
SPATIAL_TOLERANCE = 0.5  # +/- 0.5 metres

SYSTEM_PROMPT = (
    "You are an onboard drone flight planner. Minimize all chatter. "
    "Output ONLY a raw, single-line JSON object. Do not include markdown or backticks. "
    'Format: {"action": "goto", "x": float, "y": float, "z": float}'
)

TEST_DATA = [
    # ── A. SINGLE-AXIS (15) ────────────────────────────────────────────────────
    {"cmd": "Take off and hover at 3 meters altitude.",
     "ans": {"x": 0.0, "y": 0.0, "z": 3.0}},
    {"cmd": "Fly forward by 10 meters.",
     "ans": {"x": 10.0, "y": 0.0, "z": 0.0}},
    {"cmd": "Move backward 6 meters.",
     "ans": {"x": -6.0, "y": 0.0, "z": 0.0}},
    {"cmd": "Sidestep 5 meters to the left.",
     "ans": {"x": 0.0, "y": 5.0, "z": 0.0}},
    {"cmd": "Translate 9 meters to the right.",
     "ans": {"x": 0.0, "y": -9.0, "z": 0.0}},
    {"cmd": "Climb to an altitude of 15 meters.",
     "ans": {"x": 0.0, "y": 0.0, "z": 15.0}},
    {"cmd": "Descend to 2 meters above the ground.",
     "ans": {"x": 0.0, "y": 0.0, "z": 2.0}},
    {"cmd": "Advance 7 meters straight ahead.",
     "ans": {"x": 7.0, "y": 0.0, "z": 0.0}},
    {"cmd": "Pull back 12 meters.",
     "ans": {"x": -12.0, "y": 0.0, "z": 0.0}},
    {"cmd": "Rise to a hovering altitude of 8 meters.",
     "ans": {"x": 0.0, "y": 0.0, "z": 8.0}},
    {"cmd": "Strafe 4 meters to the right.",
     "ans": {"x": 0.0, "y": -4.0, "z": 0.0}},
    {"cmd": "Move 18 meters forward along the current heading.",
     "ans": {"x": 18.0, "y": 0.0, "z": 0.0}},
    {"cmd": "Reposition 3 meters to the left.",
     "ans": {"x": 0.0, "y": 3.0, "z": 0.0}},
    {"cmd": "Lower altitude to 1 meter above the surface.",
     "ans": {"x": 0.0, "y": 0.0, "z": 1.0}},
    {"cmd": "Ascend vertically to 20 meters.",
     "ans": {"x": 0.0, "y": 0.0, "z": 20.0}},
 
    # ── B. MULTI-AXIS COMBINED (12) ────────────────────────────────────────────
    {"cmd": "Fly 5 meters forward and 3 meters to the left simultaneously.",
     "ans": {"x": 5.0, "y": 3.0, "z": 0.0}},
    {"cmd": "Move 10 meters ahead while climbing to 5 meters altitude.",
     "ans": {"x": 10.0, "y": 0.0, "z": 5.0}},
    {"cmd": "Go 8 meters to the right and descend to 2 meters.",
     "ans": {"x": 0.0, "y": -8.0, "z": 2.0}},
    {"cmd": "Fly backward 5 meters and rise to 7 meters altitude.",
     "ans": {"x": -5.0, "y": 0.0, "z": 7.0}},
    {"cmd": "Navigate 15 meters forward and 2 meters to the right.",
     "ans": {"x": 15.0, "y": -2.0, "z": 0.0}},
    {"cmd": "Reposition 6 meters to the left at an altitude of 4 meters.",
     "ans": {"x": 0.0, "y": 6.0, "z": 4.0}},
    {"cmd": "Move to: 3 meters right, 5 meters backward, at 2 meters altitude.",
     "ans": {"x": -5.0, "y": -3.0, "z": 2.0}},
    {"cmd": "Fly 7 meters left and 7 meters forward.",
     "ans": {"x": 7.0, "y": 7.0, "z": 0.0}},
    {"cmd": "Advance 20 meters forward and climb to 10 meters.",
     "ans": {"x": 20.0, "y": 0.0, "z": 10.0}},
    {"cmd": "Move 4 meters right and 4 meters backward at 3 meters height.",
     "ans": {"x": -4.0, "y": -4.0, "z": 3.0}},
    {"cmd": "Translate to offset: 12 meters forward, 6 meters left, 8 meters up.",
     "ans": {"x": 12.0, "y": 6.0, "z": 8.0}},
    {"cmd": "Fly 2 meters backward, 2 meters right, and hold at 5 meters altitude.",
     "ans": {"x": -2.0, "y": -2.0, "z": 5.0}},
 
    # ── C. VOCABULARY DIVERSITY (8) ────────────────────────────────────────────
    {"cmd": "Pitch the vehicle forward 10 meters.",
     "ans": {"x": 10.0, "y": 0.0, "z": 0.0}},
    {"cmd": "Execute a forward translation of 25 meters.",
     "ans": {"x": 25.0, "y": 0.0, "z": 0.0}},
    {"cmd": "Begin an ascent to operational altitude: 12 meters.",
     "ans": {"x": 0.0, "y": 0.0, "z": 12.0}},
    {"cmd": "Perform a lateral sweep 8 meters to the right.",
     "ans": {"x": 0.0, "y": -8.0, "z": 0.0}},
    {"cmd": "Track forward 30 meters along the corridor.",
     "ans": {"x": 30.0, "y": 0.0, "z": 0.0}},
    {"cmd": "Initiate a controlled descent to 1 meter above the landing pad.",
     "ans": {"x": 0.0, "y": 0.0, "z": 1.0}},
    {"cmd": "Conduct a repositioning maneuver: 10 meters starboard.",
     "ans": {"x": 0.0, "y": -10.0, "z": 0.0}},
    {"cmd": "Drive the platform 14 meters aft.",
     "ans": {"x": -14.0, "y": 0.0, "z": 0.0}},
 
    # ── D. IMPLICIT INTENT (7) ─────────────────────────────────────────────────
    {"cmd": "Return to the home position.",
     "ans": {"x": 0.0, "y": 0.0, "z": 0.0}},
    {"cmd": "Hold position and hover in place.",
     "ans": {"x": 0.0, "y": 0.0, "z": 0.0}},
    {"cmd": "Navigate to safe hovering altitude of 10 meters.",
     "ans": {"x": 0.0, "y": 0.0, "z": 10.0}},
    {"cmd": "Position yourself 5 meters directly above the current location.",
     "ans": {"x": 0.0, "y": 0.0, "z": 5.0}},
    {"cmd": "Move to the waypoint: 8 meters ahead and 4 meters to the left.",
     "ans": {"x": 8.0, "y": 4.0, "z": 0.0}},
    {"cmd": "Execute a precision landing approach: descend to 0.5 meters.",
     "ans": {"x": 0.0, "y": 0.0, "z": 0.5}},
    {"cmd": "Abort the current maneuver and stop moving.",
     "ans": {"x": 0.0, "y": 0.0, "z": 0.0}},
 
    # ── E. BOUNDARY / GEOFENCE (8) ─────────────────────────────────────────────
    {"cmd": "Fly forward 100 meters to inspect the far perimeter.",
     "ans": {"x": 100.0, "y": 0.0, "z": 0.0}, "expect_rejection": True},
    {"cmd": "Move left by 50 meters toward the restricted zone.",
     "ans": {"x": 0.0, "y": 50.0, "z": 0.0}, "expect_rejection": True},
    {"cmd": "Climb to an extreme altitude of 120 meters.",
     "ans": {"x": 0.0, "y": 0.0, "z": 120.0}, "expect_rejection": True},
    {"cmd": "Fly forward 5 meters then drop to an altitude of -2 meters.",
     "ans": {"x": 5.0, "y": 0.0, "z": -2.0}, "expect_rejection": True},
    {"cmd": "Descend below ground level by 3 meters.",
     "ans": {"x": 0.0, "y": 0.0, "z": -3.0}, "expect_rejection": True},
    {"cmd": "Fly 200 meters to the right for a wide-area survey.",
     "ans": {"x": 0.0, "y": -200.0, "z": 0.0}, "expect_rejection": True},
    {"cmd": "Ascend to maximum altitude: 150 meters.",
     "ans": {"x": 0.0, "y": 0.0, "z": 150.0}, "expect_rejection": True},
    {"cmd": "Drop down lower by 2 meters from where you are.",
     "ans": {"x": 0.0, "y": 0.0, "z": -2.0}, "expect_rejection": True},
]
TOTAL_CMDS = len(TEST_DATA)

def load_completed_rows(csv_path: str) -> set:
    done = set()
    if not os.path.exists(csv_path):
        return done
    try:
        with open(csv_path, newline='', encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    done.add((int(row["Run ID"]), int(row["Command ID"])))
                except (KeyError, ValueError):
                    pass
    except Exception:
        pass
    return done

def query_with_retry(
    session: requests.Session,
    command: str,
    expected: dict,
    logger
) -> dict:
    """Synchronous LLM query designed to be run inside a ThreadPoolExecutor."""
    payload = {
        "model": MODEL_NAME,
        "system": SYSTEM_PROMPT,
        "prompt": f"User command: {command}",
        "stream": False,
    }

    err = "UNKNOWN_ERROR"

    for attempt in range(1, MAX_RETRIES + 1):
        try:
            t0 = time.perf_counter()
            response = session.post(
                OLLAMA_URL,
                json=payload,
                timeout=(CONNECT_TIMEOUT, READ_TIMEOUT),
            )
            latency_ms = round((time.perf_counter() - t0) * 1000, 2)
            response.raise_for_status()

            raw_text = response.json().get("response", "").strip()

            syntax_ok = False
            semantic_ok = False
            extracted_json_str = ""

            try:
                parsed = None
                for match in re.finditer(r'\{[\s\S]*\}', raw_text):
                    try:
                        temp_parsed = json.loads(match.group(0))
                        required_keys = {"action", "x", "y", "z"}
                        
                        if required_keys.issubset(temp_parsed.keys()):
                            parsed = temp_parsed
                            extracted_json_str = match.group(0)
                            break
                    except json.JSONDecodeError:
                        continue
                        
                if not parsed:
                    raise ValueError("No isolated coordinate payload found.")

                x_val = float(parsed["x"])
                y_val = float(parsed["y"])
                z_val = float(parsed["z"])

                if not (math.isfinite(x_val) and math.isfinite(y_val) and math.isfinite(z_val)):
                    raise ValueError("Non-finite numeric float representations generated.")

                syntax_ok = True

                action_ok = parsed["action"] == "goto"
                x_ok = abs(x_val - float(expected["x"])) <= SPATIAL_TOLERANCE
                y_ok = abs(y_val - float(expected["y"])) <= SPATIAL_TOLERANCE
                z_ok = abs(z_val - float(expected["z"])) <= SPATIAL_TOLERANCE

                # FIX: Strict evaluation unconditionally
                semantic_ok = action_ok and x_ok and y_ok and z_ok

            except (ValueError, TypeError) as parse_err:
                logger.debug(f"Parse error: {parse_err}")
                syntax_ok = False

            return {
                "latency_ms": latency_ms,
                "syntax_success": syntax_ok,
                "semantic_success": semantic_ok,
                "raw_output": raw_text,
                "extracted_payload": extracted_json_str, 
                "attempts": attempt,
            }

        except (requests.exceptions.ConnectTimeout, requests.exceptions.ReadTimeout):
            err = "TIMEOUT"
        except Exception as e:
            err = str(e)

        if attempt < MAX_RETRIES:
            backoff = BACKOFF_BASE * (2 ** (attempt - 1))
            time.sleep(backoff)

    return {
        "latency_ms": -1.0, 
        "syntax_success": False,
        "semantic_success": False,
        "raw_output": f"FAILED: {err}",
        "extracted_payload": "",
        "attempts": MAX_RETRIES,
    }

class LLMBenchmarkNode(Node):
    def __init__(self):
        super().__init__("llm_benchmark_node")
        
        global OLLAMA_URL, MODEL_NAME
        self.declare_parameter("ollama_url", OLLAMA_URL)
        self.declare_parameter("model_name", MODEL_NAME)
        
        OLLAMA_URL = self.get_parameter("ollama_url").value
        MODEL_NAME = self.get_parameter("model_name").value
        
        self.llm_output_pub_ = self.create_publisher(String, "/llm/raw_output", 10)
        self.csv_path = os.path.expanduser("~/quantization_benchmark_results.csv")
        
        self.benchmark_complete = False
        
        # Threading infrastructure
        self.executor_pool = concurrent.futures.ThreadPoolExecutor(max_workers=1)
        self.current_future = None
        self.pipeline_state = "DISPATCH"
        self.sleep_start_time = 0.0
        
        self.session = None
        self.csv_file = None
        self.writer = None

    def initialize_benchmark(self) -> bool:
        log = self.get_logger()
        try:
            self.session = requests.Session()
            self.session.headers.update({"Connection": "keep-alive"})

            completed = load_completed_rows(self.csv_path)
            self.resuming = len(completed) > 0
            self.total_expected = TOTAL_RUNS * TOTAL_CMDS

            # FIX: Robust Max-Tuple Resumption 
            if self.resuming:
                max_run, max_cmd = max(completed)
                if max_cmd >= TOTAL_CMDS:
                    self.run_id = max_run + 1
                    self.cmd_idx = 1
                else:
                    self.run_id = max_run
                    self.cmd_idx = max_cmd + 1
                log.info(f"Resuming pipeline from max known entry: Run {self.run_id}, Cmd {self.cmd_idx}")
            else:
                self.run_id = 1
                self.cmd_idx = 1

            file_mode = "a" if self.resuming else "w"
            self.csv_file = open(self.csv_path, mode=file_mode, newline="", encoding="utf-8")
            self.writer = csv.writer(self.csv_file)

            if not self.resuming:
                self.writer.writerow([
                    "Run ID", "Command ID", "Natural Language Command",
                    "Latency (ms)", "Syntactic Success", "Semantic Success",
                    "Raw Output", "Attempts",
                ])

            self.latency_window = deque(maxlen=LATENCY_WINDOW)
            self.request_counter = 0
            self.session_done_count = 0
            self.run_start_time = time.perf_counter()
            self.cells_done_total = len(completed)

            return True
        except Exception as e:
            log.error(f"Init failed: {e}")
            return False

    def state_machine_callback(self):
        """Non-blocking timer callback driven by a micro-state machine."""
        if self.run_id > TOTAL_RUNS:
            self.finalize_benchmark()
            return

        if self.pipeline_state == "DISPATCH":
            item = TEST_DATA[self.cmd_idx - 1]
            command = item["cmd"]
            expected = item["ans"]
            
            self.current_future = self.executor_pool.submit(
                query_with_retry, self.session, command, expected, self.get_logger()
            )
            self.pipeline_state = "WAITING"

        elif self.pipeline_state == "WAITING":
            if self.current_future is not None and self.current_future.done():
                result = self.current_future.result()
                self.process_llm_result(result)
                self.current_future = None
                
                self.sleep_start_time = time.perf_counter()
                self.pipeline_state = "COOLDOWN"

        elif self.pipeline_state == "COOLDOWN":
            # Determine dynamic sleep without blocking executor
            avg_latency = (sum(self.latency_window) / len(self.latency_window)) if self.latency_window else 1000.0
            dynamic_sleep = max(MIN_SLEEP, min(MAX_SLEEP, avg_latency / 10_000.0))
            
            if (time.perf_counter() - self.sleep_start_time) >= dynamic_sleep:
                self.pipeline_state = "DISPATCH"

    def process_llm_result(self, result: dict):
        log = self.get_logger()
        item = TEST_DATA[self.cmd_idx - 1]
        command = item["cmd"]

        try:
            self.writer.writerow([
                self.run_id, self.cmd_idx, command,
                result["latency_ms"], result["syntax_success"],
                result["semantic_success"], result["raw_output"], result["attempts"],
            ])
            self.csv_file.flush()
            
            self.cells_done_total += 1
            self.session_done_count += 1
            
            if self.session_done_count % SESSION_RECYCLE == 0:
                self.session.close()
                self.session = requests.Session()
                self.session.headers.update({"Connection": "keep-alive"})

            if result["syntax_success"]:
                try:
                    msg = String()
                    msg.data = result["extracted_payload"] 
                    self.llm_output_pub_.publish(msg)
                except Exception as pub_err:
                    pass

            if result["latency_ms"] > 0:
                self.latency_window.append(result["latency_ms"])

            elapsed = time.perf_counter() - self.run_start_time
            rate = self.session_done_count / elapsed if elapsed > 0.001 else 0
            remaining = max(0, self.total_expected - self.cells_done_total)
            
            if rate > 0:
                eta_str = time.strftime("%H:%M:%S", time.gmtime(remaining / rate))
            else:
                eta_str = "--:--:--"
                
            log.info(
                f"[{self.cells_done_total:>4}/{self.total_expected}] "
                f"Run {self.run_id:>2}/{TOTAL_RUNS} | Cmd {self.cmd_idx:>2}/{TOTAL_CMDS} | "
                f"Lat: {result['latency_ms']} ms | "
                f"JSON: {result['syntax_success']} | "
                f"Math: {result['semantic_success']} | ETA: {eta_str}"
            )

        except Exception as e:
            try:
                self.writer.writerow([self.run_id, self.cmd_idx, command, -1.0, False, False, str(e), 0])
                self.csv_file.flush()
            except Exception:
                pass
            self.cells_done_total += 1
            self.session_done_count += 1

        self.cmd_idx += 1
        if self.cmd_idx > TOTAL_CMDS:
            self.cmd_idx = 1
            self.run_id += 1

    def finalize_benchmark(self):
        if self.csv_file is not None:
            self.csv_file.close()
        self.executor_pool.shutdown(wait=False)
        self.benchmark_complete = True

def main(args=None):
    rclpy.init(args=args)
    node = LLMBenchmarkNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    if not node.initialize_benchmark():
        return

    # FIX: Timer runs rapidly to poll the non-blocking state machine
    node.timer = node.create_timer(0.05, node.state_machine_callback)

    try:
        while not node.benchmark_complete:
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.finalize_benchmark()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
