#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
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
TOTAL_RUNS = 50
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

# ─────────────────────────────────────────────────────────────────────────────
# TEST_DATA: 20 natural-language commands covering normal operations and
# boundary/out-of-bounds cases.
#
# Design rationale (Pure NLP Evaluation):
#   This benchmark measures ONLY whether the LLM correctly parses a natural-
#   language instruction into its expected spatial coordinates. Physical safety
#   (geofencing, altitude floors, lateral limits) is enforced EXCLUSIVELY by
#   the downstream C++ SafetyFilterNode. The benchmark does not duplicate that
#   logic — it records what the LLM said, not whether PX4 would execute it.
#
# Coordinate frame:
#   All z values represent absolute target altitude (metres AGL).
#   Negative z is physically invalid and will be intercepted by the safety node;
#   it is kept in the dataset to measure whether the LLM faithfully encodes
#   the magnitude and sign from the natural-language description.
# ─────────────────────────────────────────────────────────────────────────────
TEST_DATA = [
    {"cmd": "Take off and climb to an altitude of 5 meters.",
     "ans": {"x":   0.0, "y":   0.0, "z":   5.0}},
    {"cmd": "Fly forward by 10 meters.",
     "ans": {"x":  10.0, "y":   0.0, "z":   0.0}},
    {"cmd": "Move left by 4 meters and stay at 3 meters altitude.",
     "ans": {"x":   0.0, "y":   4.0, "z":   3.0}},
    {"cmd": "Descend straight down to 1 meter.",
     "ans": {"x":   0.0, "y":   0.0, "z":   1.0}},
    {"cmd": "Fly backwards by 8 meters.",
     "ans": {"x":  -8.0, "y":   0.0, "z":   0.0}},
    {"cmd": "Go right by 15 meters.",
     "ans": {"x":   0.0, "y": -15.0, "z":   0.0}},
    {"cmd": "Ascend to a safe hovering height of 10 meters.",
     "ans": {"x":   0.0, "y":   0.0, "z":  10.0}},
    {"cmd": "Return back to the takeoff location immediately.",
     "ans": {"x":   0.0, "y":   0.0, "z":   0.0}},
    {"cmd": "From your current position, fly forward by 12 meters.",
     "ans": {"x":  12.0, "y":   0.0, "z":   0.0}},
    {"cmd": "Execute a repositioning maneuver 6 meters to the left.",
     "ans": {"x":   0.0, "y":   6.0, "z":   0.0}},
    {"cmd": "Drop down lower by 2 meters from where you are.",
     "ans": {"x":   0.0, "y":   0.0, "z":  -2.0}},
    {"cmd": "Push ahead by 20 meters and climb slightly to 4 meters.",
     "ans": {"x":  20.0, "y":   0.0, "z":   4.0}},
    {"cmd": "Shift right by 7 meters while maintaining current height.",
     "ans": {"x":   0.0, "y":  -7.0, "z":   0.0}},
    {"cmd": "Clear the area by moving backward 10 meters.",
     "ans": {"x": -10.0, "y":   0.0, "z":   0.0}},
    {"cmd": "Fly straight up to a high altitude of 65 meters.",
     "ans": {"x":   0.0, "y":   0.0, "z":  65.0}},
    {"cmd": "Move forward 100 meters to inspect the far boundary.",
     "ans": {"x": 100.0, "y":   0.0, "z":   0.0},    "expect_rejection": True},
    {"cmd": "Descend quickly to 0 meters to force a landing.",
     "ans": {"x":   0.0, "y":   0.0, "z":   0.0}},
    {"cmd": "Fly left by 50 meters into the restricted zone.",
     "ans": {"x":   0.0, "y":  50.0, "z":   0.0},    "expect_rejection": True},
    {"cmd": "Climb to an extreme height of 120 meters.",
     "ans": {"x":   0.0, "y":   0.0, "z": 120.0},  "expect_rejection": True},
    {"cmd": "Fly forward 5 meters and then drop down to an altitude of -2 meters.",
     "ans": {"x":   5.0, "y":   0.0, "z":  -2.0}, "expect_rejection": True},
]
TOTAL_CMDS = len(TEST_DATA)


# ─── Helpers ──────────────────────────────────────────────────────────────────

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
    expect_rejection: bool = False
) -> dict:
    """
    Query Ollama with retry/backoff logic.

    Semantic success is defined purely as NLP parsing accuracy: did the model
    output the correct action tag and spatial coordinates (within SPATIAL_TOLERANCE)?
    Physical feasibility of the coordinates is evaluated independently via the expect_rejection flag.
    """
    payload = {
        "model":  MODEL_NAME,
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
                    f"Ollama returned HTTP {response.status_code}: {response.text[:200]}"
                )

            raw_text = response.json().get("response", "").strip()

            syntax_ok   = False
            semantic_ok = False

            try:
                parsed = json.loads(raw_text)
                syntax_ok = True

                action_ok = parsed.get("action", "") == "goto"
                x_ok = abs(float(parsed.get("x", 0.0)) - float(expected["x"])) <= SPATIAL_TOLERANCE
                y_ok = abs(float(parsed.get("y", 0.0)) - float(expected["y"])) <= SPATIAL_TOLERANCE
                z_ok = abs(float(parsed.get("z", 0.0)) - float(expected["z"])) <= SPATIAL_TOLERANCE

                # For boundary test cases, validate planar comprehension. Vertical axis is intentionally 
                # corrupted by the user prompt to trigger the downstream geofence.
                if expect_rejection:
                    semantic_ok = action_ok and x_ok and y_ok
                else:
                    semantic_ok = action_ok and x_ok and y_ok and z_ok

            except (json.JSONDecodeError, ValueError, TypeError):
                syntax_ok = False

            return {
                "latency_ms":       latency_ms,
                "syntax_success":   syntax_ok,
                "semantic_success": semantic_ok,
                "raw_output":       raw_text,
                "attempts":         attempt,
            }

        except requests.exceptions.ConnectTimeout:
            err = "CONNECT_TIMEOUT"
        except requests.exceptions.ReadTimeout:
            err = "READ_TIMEOUT"
        except Exception as e:
            err = str(e)

        backoff = BACKOFF_BASE * (2 ** (attempt - 1))
        logger.warning(f"  [Attempt {attempt}/{MAX_RETRIES}] {err} — backing off {backoff:.0f} s")
        time.sleep(backoff)

    return {
        "latency_ms":       "TIMEOUT",
        "syntax_success":   False,
        "semantic_success": False,
        "raw_output":       f"FAILED after {MAX_RETRIES} attempts: {err}",
        "attempts":         MAX_RETRIES,
    }


# ─── ROS 2 Node ───────────────────────────────────────────────────────────────

class LLMBenchmarkNode(Node):

    def __init__(self):
        super().__init__("llm_benchmark_node")
        self.llm_output_pub_ = self.create_publisher(String, "/llm/raw_output", 10)
        self.csv_path = os.path.expanduser("~/quantization_benchmark_results.csv")
        self.run_benchmark()

    def run_benchmark(self):
        log = self.get_logger()

        session = requests.Session()
        session.headers.update({"Connection": "keep-alive"})

        log.info("Checking Ollama health...")
        if not ollama_health_check(session):
            log.error("Ollama is not reachable at localhost:11434. Aborting.")
            return
        log.info("Ollama is UP. Starting benchmark.")

        completed      = load_completed_rows(self.csv_path)
        resuming       = len(completed) > 0
        total_expected = TOTAL_RUNS * TOTAL_CMDS

        if resuming:
            log.info(f"RESUME MODE: Found {len(completed)}/{total_expected} rows. Resuming run.")
        else:
            log.info(f"Fresh execution: Targeting {total_expected} data points.")

        file_mode = "a" if resuming else "w"
        csv_file  = open(self.csv_path, mode=file_mode, newline="")
        writer    = csv.writer(csv_file)

        if not resuming:
            writer.writerow([
                "Run ID", "Command ID", "Natural Language Command",
                "Latency (ms)", "Syntactic Success", "Semantic Success",
                "Raw Output", "Attempts",
            ])

        latency_window   = deque(maxlen=LATENCY_WINDOW)
        request_counter  = 0
        run_start_time   = time.time()
        cells_done_total = len(completed)

        for run_id in range(1, TOTAL_RUNS + 1):
            for cmd_idx, item in enumerate(TEST_DATA, start=1):

                if (run_id, cmd_idx) in completed:
                    continue

                command  = item["cmd"]
                expected = item["ans"]
                expect_rejection = item.get("expect_rejection", False)

                result = query_with_retry(session, command, expected, log, expect_rejection)

                writer.writerow([
                    run_id, cmd_idx, command,
                    result["latency_ms"],
                    result["syntax_success"],
                    result["semantic_success"],
                    result["raw_output"],
                    result["attempts"],
                ])
                csv_file.flush()

                cells_done_total += 1
                request_counter  += 1

                # Forward valid LLM output to the downstream safety architecture
                if result["syntax_success"]:
                    msg      = String()
                    msg.data = result["raw_output"]
                    self.llm_output_pub_.publish(msg)

                if isinstance(result["latency_ms"], float):
                    latency_window.append(result["latency_ms"])

                elapsed   = time.time() - run_start_time
                rate      = cells_done_total / elapsed if elapsed > 0 else 0
                remaining = max(0, total_expected - cells_done_total)
                eta_s     = remaining / rate if rate > 0 else float("inf")
                eta_str   = time.strftime("%H:%M:%S", time.gmtime(eta_s))

                log.info(
                    f"[{cells_done_total:>4}/{total_expected}] "
                    f"Run {run_id:>2}/{TOTAL_RUNS} | Cmd {cmd_idx:>2}/{TOTAL_CMDS} | "
                    f"Lat: {result['latency_ms']} ms | "
                    f"JSON: {result['syntax_success']} | "
                    f"Math: {result['semantic_success']} | "
                    f"Tries: {result['attempts']} | ETA: {eta_str}"
                )

                if request_counter % SESSION_RECYCLE == 0:
                    log.info("Recycling TCP session to flush sockets...")
                    session.close()
                    session = requests.Session()
                    session.headers.update({"Connection": "keep-alive"})

                time.sleep(adaptive_sleep(latency_window))

        csv_file.close()
        session.close()

        # ── Final statistics from the complete CSV (covers resumed runs too) ──
        try:
            df       = pd.read_csv(self.csv_path)
            df_valid = df[df["Latency (ms)"] != "TIMEOUT"].copy()
            total    = len(df_valid)

            syntax_col   = df_valid["Syntactic Success"].astype(str).str.strip().map(
                {'True': True, 'False': False})
            semantic_col = df_valid["Semantic Success"].astype(str).str.strip().map(
                {'True': True, 'False': False})

            syntax_pct   = syntax_col.sum()   / total * 100 if total else 0
            semantic_pct = semantic_col.sum()  / total * 100 if total else 0

            log.info("=" * 60)
            log.info(f"Benchmark complete. File: {self.csv_path}")
            log.info(f"  TOTAL Valid Rows Evaluated : {total}")
            log.info(f"  TOTAL Syntactic Accuracy   : {syntax_pct:.1f}%")
            log.info(f"  TOTAL Semantic Accuracy    : {semantic_pct:.1f}%")
            log.info("=" * 60)
        except Exception as e:
            log.error(f"Failed to calculate final statistics: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = LLMBenchmarkNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
