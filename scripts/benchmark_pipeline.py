import rclpy
from rclpy.node import Node
import requests
import json
import time
import csv
import os
from collections import deque
from geometry_msgs.msg import PoseStamped

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

SYSTEM_PROMPT = (
    "You are an onboard drone flight planner. Minimize all chatter. "
    "Output ONLY a raw, single-line JSON object. Do not include markdown or backticks. "
    'Format: {"action": "goto", "x": float, "y": float, "z": float}'
)

TEST_DATA = [
    {"cmd": "Take off and climb to an altitude of 5 meters.",              "ans": {"x": 0.0,   "y": 0.0,   "z": 5.0}},
    {"cmd": "Fly forward by 10 meters.",                                    "ans": {"x": 10.0,  "y": 0.0,   "z": 0.0}},
    {"cmd": "Move left by 4 meters and stay at 3 meters altitude.",         "ans": {"x": 0.0,   "y": 4.0,   "z": 3.0}},
    {"cmd": "Descend straight down to 1 meter.",                            "ans": {"x": 0.0,   "y": 0.0,   "z": 1.0}},
    {"cmd": "Fly backwards by 8 meters.",                                   "ans": {"x": -8.0,  "y": 0.0,   "z": 0.0}},
    {"cmd": "Go right by 15 meters.",                                       "ans": {"x": 0.0,   "y": -15.0, "z": 0.0}},
    {"cmd": "Ascend to a safe hovering height of 10 meters.",               "ans": {"x": 0.0,   "y": 0.0,   "z": 10.0}},
    {"cmd": "Return back to the takeoff location immediately.",             "ans": {"x": 0.0,   "y": 0.0,   "z": 0.0}},
    {"cmd": "From your current position, fly forward by 12 meters.",        "ans": {"x": 12.0,  "y": 0.0,   "z": 0.0}},
    {"cmd": "Execute a repositioning maneuver 6 meters to the left.",       "ans": {"x": 0.0,   "y": 6.0,   "z": 0.0}},
    {"cmd": "Drop down lower by 2 meters from where you are.",              "ans": {"x": 0.0,   "y": 0.0,   "z": -2.0}},
    {"cmd": "Push ahead by 20 meters and climb slightly to 4 meters.",      "ans": {"x": 20.0,  "y": 0.0,   "z": 4.0}},
    {"cmd": "Shift right by 7 meters while maintaining current height.",    "ans": {"x": 0.0,   "y": -7.0,  "z": 0.0}},
    {"cmd": "Clear the area by moving backward 10 meters.",                 "ans": {"x": -10.0, "y": 0.0,   "z": 0.0}},
    {"cmd": "Fly straight up to a high altitude of 65 meters.",             "ans": {"x": 0.0,   "y": 0.0,   "z": 65.0}},
    {"cmd": "Move forward 100 meters to inspect the far boundary.",         "ans": {"x": 100.0, "y": 0.0,   "z": 0.0}},
    {"cmd": "Descend quickly to 0 meters to force a landing.",              "ans": {"x": 0.0,   "y": 0.0,   "z": 0.0}},
    {"cmd": "Fly left by 50 meters into the restricted zone.",              "ans": {"x": 0.0,   "y": 50.0,  "z": 0.0}},
    {"cmd": "Climb to an extreme height of 120 meters.",                    "ans": {"x": 0.0,   "y": 0.0,   "z": 120.0}},
    {"cmd": "Fly forward 5 meters and then drop down to an altitude of -2 meters.", "ans": {"x": 5.0, "y": 0.0, "z": -2.0}},
]
TOTAL_CMDS = len(TEST_DATA)

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
    sleep_s = max(MIN_SLEEP, min(MAX_SLEEP, avg_ms / 10_000))
    return sleep_s

def query_with_retry(session: requests.Session, command: str, expected: dict, logger) -> dict:
    payload = {
        "model":  MODEL_NAME,
        "prompt": f"{SYSTEM_PROMPT}\nUser command: {command}",
        "stream": False,
    }

    for attempt in range(1, MAX_RETRIES + 1):
        try:
            t0 = time.perf_counter()
            response = session.post(
                OLLAMA_URL,
                json=payload,
                timeout=(CONNECT_TIMEOUT, READ_TIMEOUT),
            )
            latency_ms = round((time.perf_counter() - t0) * 1000, 2)
            raw_text = response.json().get("response", "").strip()

            syntax_ok = False
            semantic_ok = False

            try:
                parsed = json.loads(raw_text)
                syntax_ok = True

                x_ok = abs(float(parsed.get("x", 0.0)) - float(expected["x"])) < 1e-6
                y_ok = abs(float(parsed.get("y", 0.0)) - float(expected["y"])) < 1e-6
                z_ok = abs(float(parsed.get("z", 0.0)) - float(expected["z"])) < 1e-6
                semantic_ok = x_ok and y_ok and z_ok

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

class LLMBenchmarkNode(Node):

    def __init__(self):
        super().__init__("llm_benchmark_node")
        self.publisher_ = self.create_publisher(PoseStamped, "/fmu/in/trajectory_setpoint", 10)
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

        completed = load_completed_rows(self.csv_path)
        resuming = len(completed) > 0
        total_expected = TOTAL_RUNS * TOTAL_CMDS

        if resuming:
            log.info(f"RESUME MODE: Found {len(completed)}/{total_expected} rows. Resuming run.")
        else:
            log.info(f"Fresh execution: Targeting {total_expected} data points.")

        file_mode = "a" if resuming else "w"
        csv_file = open(self.csv_path, mode=file_mode, newline="")
        writer = csv.writer(csv_file)

        if not resuming:
            writer.writerow([
                "Run ID", "Command ID", "Natural Language Command",
                "Latency (ms)", "Syntactic Success", "Semantic Success",
                "Raw Output", "Attempts"
            ])

        latency_window = deque(maxlen=LATENCY_WINDOW)
        request_counter = 0
        run_start_time = time.time()
        syntax_total = 0
        semantic_total = 0
        cells_done_total = len(completed)

        for run_id in range(1, TOTAL_RUNS + 1):
            for cmd_idx, item in enumerate(TEST_DATA, start=1):

                if (run_id, cmd_idx) in completed:
                    continue

                command = item["cmd"]
                expected = item["ans"]

                result = query_with_retry(session, command, expected, log)

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
                request_counter += 1

                if isinstance(result["latency_ms"], float):
                    latency_window.append(result["latency_ms"])
                    syntax_total += int(result["syntax_success"])
                    semantic_total += int(result["semantic_success"])

                elapsed = time.time() - run_start_time
                rate = cells_done_total / elapsed if elapsed > 0 else 0
                remaining = max(0, total_expected - cells_done_total)
                eta_s = remaining / rate if rate > 0 else float("inf")
                eta_str = time.strftime("%H:%M:%S", time.gmtime(eta_s))

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

                sleep_s = adaptive_sleep(latency_window)
                time.sleep(sleep_s)

        csv_file.close()
        session.close()

        valid_cells = request_counter  
        syntax_pct = (syntax_total / valid_cells * 100) if valid_cells else 0
        semantic_pct = (semantic_total / valid_cells * 100) if valid_cells else 0

        log.info("=" * 60)
        log.info(f"Benchmark complete. File location: {self.csv_path}")
        log.info(f"  Total processed this session : {valid_cells}")
        log.info(f"  Syntactic Session Accuracy   : {syntax_pct:.1f}%")
        log.info(f"  Semantic Session Accuracy    : {semantic_pct:.1f}%")
        log.info("=" * 60)

def main(args=None):
    rclpy.init(args=args)
    node = LLMBenchmarkNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()