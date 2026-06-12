#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import requests
import json
import time
import csv
import os
import pandas as pd
from collections import deque
from std_msgs.msg import String

OLLAMA_URL = "http://localhost:11434/api/generate"
MODEL_NAME = "qwen2.5:3b"
TOTAL_RUNS = 20
TOTAL_CMDS = 50
MAX_RETRIES = 3
BACKOFF_BASE = 3.0
CONNECT_TIMEOUT = 5
READ_TIMEOUT = 90
SESSION_RECYCLE = 50
LATENCY_WINDOW = 10
MIN_SLEEP = 0.5
MAX_SLEEP = 6.0
SPATIAL_TOLERANCE = 0.5

SYSTEM_PROMPT = (
    "You are an onboard drone flight planner. Minimize all chatter. "
    "Output ONLY a raw, single-line JSON object. Do not include markdown or backticks. "
    'Format: {"action": "goto", "x": float, "y": float, "z": float}'
)

# [KEEP YOUR FULL TEST_DATA LIST HERE]
TEST_DATA = [
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
    # Tests: Can the model correctly decompose multi-component spatial instructions?
 
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
    # Tests: Does the model understand synonyms and domain-specific language?
 
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
    # Tests: Can the model infer numeric intent from non-explicit descriptions?
 
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
    # Tests: Does the LLM correctly encode out-of-bounds values?
    # NLP accuracy: did the LLM parse the number correctly?
    # Physical safety: the C++ safety filter will reject these at runtime.
 
    {"cmd": "Fly forward 100 meters to inspect the far perimeter.",
     "ans": {"x": 100.0, "y": 0.0, "z": 0.0},
     "expect_rejection": True},
 
    {"cmd": "Move left by 50 meters toward the restricted zone.",
     "ans": {"x": 0.0, "y": 50.0, "z": 0.0},
     "expect_rejection": True},
 
    {"cmd": "Climb to an extreme altitude of 120 meters.",
     "ans": {"x": 0.0, "y": 0.0, "z": 120.0},
     "expect_rejection": True},
 
    {"cmd": "Fly forward 5 meters then drop to an altitude of -2 meters.",
     "ans": {"x": 5.0, "y": 0.0, "z": -2.0},
     "expect_rejection": True},
 
    {"cmd": "Descend below ground level by 3 meters.",
     "ans": {"x": 0.0, "y": 0.0, "z": -3.0},
     "expect_rejection": True},
 
    {"cmd": "Fly 200 meters to the right for a wide-area survey.",
     "ans": {"x": 0.0, "y": -200.0, "z": 0.0},
     "expect_rejection": True},
 
    {"cmd": "Ascend to maximum altitude: 150 meters.",
     "ans": {"x": 0.0, "y": 0.0, "z": 150.0},
     "expect_rejection": True},
 
    {"cmd": "Drop down lower by 2 meters from where you are.",
     "ans": {"x": 0.0, "y": 0.0, "z": -2.0},
     "expect_rejection": True},
]

# ─── Helper Functions ─────────────────────────────────────────────────────────

def load_completed_rows(csv_path: str) -> set:
    done = set()
    if not os.path.exists(csv_path):
        return done
    try:
        with open(csv_path, newline='') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    done.add((int(row["Run ID"]), int(row["Command ID"])))
                except (KeyError, ValueError):
                    pass
    except Exception:
        pass
    return done

def ollama_health_check(session: requests.Session) -> bool:
    try:
        r = session.get("http://localhost:11434/", timeout=(CONNECT_TIMEOUT, 5))
        return r.status_code == 200
    except Exception:
        return False

def adaptive_sleep(latency_window: deque) -> float:
    if not latency_window:
        return MIN_SLEEP
    avg_ms = sum(latency_window) / len(latency_window)
    return max(MIN_SLEEP, min(MAX_SLEEP, avg_ms / 10_000.0))

def query_with_retry(
    session: requests.Session,
    command: str,
    expected: dict,
    logger,
    expect_rejection: bool = False,
) -> dict:
    """Synchronous LLM query with strict JSON validation."""
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

            if response.status_code != 200:
                raise requests.exceptions.RequestException(
                    f"Ollama HTTP {response.status_code}"
                )

            raw_text = response.json().get("response", "").strip()

            syntax_ok = False
            semantic_ok = False

            try:
                parsed = json.loads(raw_text)
                
                # BUG #3 & #4 FIX: Strict Key Validation
                required_keys = {"action", "x", "y", "z"}
                if not required_keys.issubset(parsed.keys()):
                    raise ValueError(f"Missing keys: {required_keys - parsed.keys()}")
                if not set(parsed.keys()).issubset(required_keys):
                     raise ValueError(f"Unexpected extra keys present.")
                
                syntax_ok = True

                action_ok = parsed["action"] == "goto"
                x_ok = abs(float(parsed["x"]) - float(expected["x"])) <= SPATIAL_TOLERANCE
                y_ok = abs(float(parsed["y"]) - float(expected["y"])) <= SPATIAL_TOLERANCE
                z_ok = abs(float(parsed["z"]) - float(expected["z"])) <= SPATIAL_TOLERANCE

                if expect_rejection:
                    semantic_ok = action_ok and x_ok and y_ok
                else:
                    semantic_ok = action_ok and x_ok and y_ok and z_ok

            except (json.JSONDecodeError, ValueError, TypeError) as parse_err:
                # Log the parse error but don't crash; mark as syntax failure
                logger.debug(f"Parse error: {parse_err}")
                syntax_ok = False

            return {
                "latency_ms": latency_ms,
                "syntax_success": syntax_ok,
                "semantic_success": semantic_ok,
                "raw_output": raw_text,
                "attempts": attempt,
            }

        except (requests.exceptions.ConnectTimeout, requests.exceptions.ReadTimeout) as timeout_err:
            err = "TIMEOUT"
        except Exception as e:
            err = str(e)

        backoff = BACKOFF_BASE * (2 ** (attempt - 1))
        logger.warning(f"  [Attempt {attempt}/{MAX_RETRIES}] {err} — backing off {backoff:.0f} s")
        time.sleep(backoff)

    return {
        "latency_ms": "TIMEOUT",
        "syntax_success": False,
        "semantic_success": False,
        "raw_output": f"FAILED: {err}",
        "attempts": MAX_RETRIES,
    }


# ─── ROS 2 Node ───────────────────────────────────────────────────────────────

class LLMBenchmarkNode(Node):
    def __init__(self):
        super().__init__("llm_benchmark_node")
        self.llm_output_pub_ = self.create_publisher(String, "/llm/raw_output", 10)
        self.csv_path = os.path.expanduser("~/quantization_benchmark_results.csv")
        
        self.benchmark_started = False
        self.benchmark_complete = False
        self.timer = None
        
        self.session = None
        self.csv_file = None
        self.writer = None

    def initialize_benchmark(self) -> bool:
        log = self.get_logger()
        try:
            self.session = requests.Session()
            self.session.headers.update({"Connection": "keep-alive"})

            if not ollama_health_check(self.session):
                log.error("Ollama is not reachable. Aborting.")
                return False

            completed = load_completed_rows(self.csv_path)
            self.resuming = len(completed) > 0
            self.total_expected = TOTAL_RUNS * TOTAL_CMDS

            if self.resuming:
                completed_count = len(completed)
                self.run_id = (completed_count // TOTAL_CMDS) + 1
                self.cmd_idx = (completed_count % TOTAL_CMDS) + 1
                log.info(f"Resuming Run {self.run_id}, Cmd {self.cmd_idx}")
            else:
                self.run_id = 1
                self.cmd_idx = 1

            file_mode = "a" if self.resuming else "w"
            self.csv_file = open(self.csv_path, mode=file_mode, newline="")
            self.writer = csv.writer(self.csv_file)

            if not self.resuming:
                self.writer.writerow([
                    "Run ID", "Command ID", "Natural Language Command",
                    "Latency (ms)", "Syntactic Success", "Semantic Success",
                    "Raw Output", "Attempts",
                ])

            self.latency_window = deque(maxlen=LATENCY_WINDOW)
            self.request_counter = 0
            self.run_start_time = time.time()
            self.cells_done_total = len(completed)

            return True

        except Exception as e:
            log.error(f"Initialization failed: {e}")
            return False

    def run_benchmark_one_command(self):
        log = self.get_logger()
        command = "UNKNOWN"
        try:
            if self.run_id > TOTAL_RUNS:
                self.finalize_benchmark()
                return

            item = TEST_DATA[self.cmd_idx - 1]
            command = item["cmd"]
            expected = item["ans"]
            expect_rejection = item.get("expect_rejection", False)

            result = query_with_retry(self.session, command, expected, log, expect_rejection)

            self.writer.writerow([
                self.run_id, self.cmd_idx, command,
                result["latency_ms"], result["syntax_success"],
                result["semantic_success"], result["raw_output"], result["attempts"],
            ])
            self.csv_file.flush()
            self.cells_done_total += 1

            if result["syntax_success"]:
                try:
                    msg = String()
                    msg.data = result["raw_output"]
                    self.llm_output_pub_.publish(msg)
                except Exception as pub_err:
                    log.warning(f"Publish failed: {pub_err}")

            if isinstance(result["latency_ms"], float):
                self.latency_window.append(result["latency_ms"])

            elapsed = time.time() - self.run_start_time
            rate = self.cells_done_total / elapsed if elapsed > 0 else 0
            remaining = max(0, self.total_expected - self.cells_done_total)
            eta_s = remaining / rate if rate > 0 else float("inf")
            eta_str = time.strftime("%H:%M:%S", time.gmtime(eta_s))
            log.info(
                f"[{self.cells_done_total:>4}/{self.total_expected}] "
                f"Run {self.run_id:>2}/{TOTAL_RUNS} | Cmd {self.cmd_idx:>2}/{TOTAL_CMDS} | "
                f"Lat: {result['latency_ms']} ms | "
                f"JSON: {result['syntax_success']} | "
                f"Math: {result['semantic_success']} | "
                f"ETA: {eta_str}"
            )

        except Exception as e:
            log.error(f"Unexpected error processing command {self.cmd_idx}: {e}")
            self.writer.writerow([self.run_id, self.cmd_idx, command, "ERROR", False, False, str(e), 0])
            self.csv_file.flush()
            self.cells_done_total += 1

        # ── Always advance, regardless of success/error ───────────────────────────
        self.cmd_idx += 1
        if self.cmd_idx > TOTAL_CMDS:
            self.cmd_idx = 1
            self.run_id += 1

        # ── TCP recycle (moved out of try block so errors don't skip it) ──────────
        self.request_counter += 1
        if self.request_counter % SESSION_RECYCLE == 0:
            log.info(f"[Session recycle] req={self.request_counter} — creating fresh TCP session.")
            self.session.close()
            self.session = requests.Session()
            self.session.headers.update({"Connection": "keep-alive"})
            if not ollama_health_check(self.session):
                log.warning("Ollama unreachable immediately after session recycle.")

        try:
            if self.timer is not None:
                self.destroy_timer(self.timer)
            sleep_duration = adaptive_sleep(self.latency_window)
            self.timer = self.create_timer(sleep_duration, self.run_benchmark_one_command)
        except Exception as timer_err:
            log.error(f"Timer fail: {timer_err}")
            self.finalize_benchmark()

    def finalize_benchmark(self):
        log = self.get_logger()
        try:
            if self.timer is not None:
                self.destroy_timer(self.timer)
            if self.csv_file is not None:
                self.csv_file.close()
            if self.session is not None:
                self.session.close()

            try:
                df = pd.read_csv(self.csv_path)
                df_valid = df[~df["Latency (ms)"].isin(["TIMEOUT", "ERROR"])].copy()
                total = len(df_valid)
                if total > 0:
                    syntax_pct = (df_valid["Syntactic Success"] == 'True').sum() / total * 100
                    semantic_pct = (df_valid["Semantic Success"] == 'True').sum() / total * 100
                    log.info("=" * 60)
                    log.info(f"Benchmark complete. File: {self.csv_path}")
                    log.info(f"  Valid Rows: {total} | Syntax: {syntax_pct:.1f}% | Semantic: {semantic_pct:.1f}%")
                    log.info("=" * 60)
            except Exception as stats_err:
                log.error(f"Stats calculation failed: {stats_err}")
        finally:
            self.benchmark_complete = True

def main(args=None):
    rclpy.init(args=args)
    node = LLMBenchmarkNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        if not node.initialize_benchmark():
            return
        node.benchmark_started = True
        node.timer = node.create_timer(0.01, node.run_benchmark_one_command)
        while not node.benchmark_complete:
            executor.spin_once(timeout_sec=1.0)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user.")
    finally:
        node.finalize_benchmark()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
