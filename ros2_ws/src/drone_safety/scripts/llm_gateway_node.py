#!/usr/bin/env python3
"""
llm_gateway_node.py
-------------------
Production runtime LLM inference node for the UAV edge AI gateway.

Subscribes to:
    /llm/command_input  (std_msgs/String) — raw natural language command

Publishes to:
    /llm/raw_output     (std_msgs/String) — validated JSON setpoint
    /llm/status         (std_msgs/String) — inference status for monitoring

Pipeline per command:
    1. Receive NL command on /llm/command_input
    2. Submit to Ollama via ThreadPoolExecutor (non-blocking)
    3. Validate JSON syntax + coordinate finiteness
    4. Publish extracted JSON to /llm/raw_output
    5. Publish status string to /llm/status

The safety_filter_node performs all geofence and kinematic validation
downstream. This node only handles LLM I/O and JSON syntax checking.

Launch:
    ros2 launch drone_safety full_system.launch.py
    ros2 run drone_safety llm_gateway_node

Manual test (in a second terminal):
    ros2 topic pub /llm/command_input std_msgs/String "data: 'Fly forward 5 meters'"
    ros2 topic echo /llm/raw_output
    ros2 topic echo /llm/status
"""

import re
import json
import math
import time
import concurrent.futures

import requests
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

# ── Default constants (overridden by ROS 2 parameters / geofence_params.yaml) ─
DEFAULT_OLLAMA_URL     = "http://localhost:11434/api/generate"
DEFAULT_MODEL_NAME     = "qwen2.5:3b"
DEFAULT_MAX_RETRIES    = 3
DEFAULT_BACKOFF_BASE   = 3.0
DEFAULT_CONNECT_TIMEOUT = 5
DEFAULT_READ_TIMEOUT   = 90

SYSTEM_PROMPT = (
    "You are an onboard drone flight planner. Minimize all chatter. "
    "Output ONLY a raw, single-line JSON object. Do not include markdown or backticks. "
    'Format: {"action": "goto", "x": float, "y": float, "z": float}\n\n'
    "Coordinate convention (body-relative FLU frame):\n"
    "  x = forward (+) / backward (-) in metres\n"
    "  y = left (+) / right (-) in metres\n"
    "  z = up (+) / down (-) in metres\n"
    "  All values are relative deltas from current position.\n\n"
    'EXAMPLES:\n'
    'User command: Climb by 5 meters.\n'
    '{"action": "goto", "x": 0.0, "y": 0.0, "z": 5.0}\n'
    'User command: Fly 10 meters forward and 3 meters to the left.\n'
    '{"action": "goto", "x": 10.0, "y": 3.0, "z": 0.0}\n'
    'User command: Drop down by 2 meters.\n'
    '{"action": "goto", "x": 0.0, "y": 0.0, "z": -2.0}'
)


def ollama_health_check(session: requests.Session, url: str,
                         connect_timeout: int) -> bool:
    """Check Ollama is reachable before accepting any commands."""
    try:
        base_url = url.rsplit("/api/", 1)[0]
        r = session.get(base_url, timeout=(connect_timeout, 5))
        return r.status_code == 200
    except Exception:
        return False


def query_ollama(
    session: requests.Session,
    command: str,
    ollama_url: str,
    model_name: str,
    max_retries: int,
    backoff_base: float,
    connect_timeout: int,
    read_timeout: int,
    logger,
) -> dict:
    """
    Synchronous LLM query — designed to run inside a ThreadPoolExecutor.
    Returns a result dict regardless of success or failure.
    """
    payload = {
        "model": model_name,
        "system": SYSTEM_PROMPT,
        "prompt": f"User command: {command}",
        "stream": False,
    }

    err = "UNKNOWN_ERROR"

    for attempt in range(1, max_retries + 1):
        try:
            t0 = time.perf_counter()
            response = session.post(
                ollama_url,
                json=payload,
                timeout=(connect_timeout, read_timeout),
            )
            latency_ms = round((time.perf_counter() - t0) * 1000, 2)
            response.raise_for_status()

            raw_text = response.json().get("response", "").strip()
            syntax_ok = False
            extracted_json_str = ""

            try:
                parsed = None
                for match in re.finditer(r'\{.*?\}', raw_text, re.DOTALL):
                    try:
                        temp = json.loads(match.group(0))
                        required = {"action", "x", "y", "z"}
                        if set(temp.keys()) == required:
                            parsed = temp
                            extracted_json_str = match.group(0)
                            break
                    except json.JSONDecodeError:
                        continue

                if not parsed:
                    raise ValueError("No valid JSON payload found in response.")

                x = float(parsed["x"])
                y = float(parsed["y"])
                z = float(parsed["z"])

                if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                    raise ValueError("Non-finite coordinate in LLM output.")

                if parsed["action"] != "goto":
                    raise ValueError(f"Unexpected action: {parsed['action']}")

                syntax_ok = True

            except (ValueError, TypeError) as parse_err:
                logger.warning(f"Parse error: {parse_err} | Raw: {raw_text[:120]}")

            return {
                "syntax_ok":        syntax_ok,
                "extracted_payload": extracted_json_str,
                "raw_output":       raw_text,
                "latency_ms":       latency_ms,
                "attempts":         attempt,
                "error":            "" if syntax_ok else str(parse_err if 'parse_err' in dir() else "parse failed"),
            }

        except (requests.exceptions.ConnectTimeout,
                requests.exceptions.ReadTimeout):
            err = "TIMEOUT"
        except requests.exceptions.RequestException as e:
            err = f"HTTP_ERROR: {e}"
        except Exception as e:
            err = str(e)

        if attempt < max_retries:
            backoff = backoff_base * (2 ** (attempt - 1))
            logger.warning(
                f"[Attempt {attempt}/{max_retries}] {err} — retrying in {backoff:.0f}s"
            )
            time.sleep(backoff)

    return {
        "syntax_ok":         False,
        "extracted_payload": "",
        "raw_output":        f"FAILED: {err}",
        "latency_ms":        -1.0,
        "attempts":          max_retries,
        "error":             err,
    }


class LLMGatewayNode(Node):
    """
    Production LLM gateway node.

    State machine (same non-blocking pattern as benchmark_pipeline):
        IDLE    — waiting for a command on /llm/command_input
        BUSY    — command received, future submitted to thread pool
        WAITING — polling the future each timer tick

    Commands received while BUSY are dropped with a warning logged to
    /llm/status. The operator must wait for the current inference to
    complete before sending the next command.
    """

    def __init__(self):
        super().__init__("llm_gateway_node")

        # ── Declare parameters (loaded from geofence_params.yaml at launch) ───
        self.declare_parameter("ollama_url",      DEFAULT_OLLAMA_URL)
        self.declare_parameter("model_name",      DEFAULT_MODEL_NAME)
        self.declare_parameter("max_retries",     DEFAULT_MAX_RETRIES)
        self.declare_parameter("backoff_base",    DEFAULT_BACKOFF_BASE)
        self.declare_parameter("connect_timeout", DEFAULT_CONNECT_TIMEOUT)
        self.declare_parameter("read_timeout",    DEFAULT_READ_TIMEOUT)

        self.ollama_url      = self.get_parameter("ollama_url").as_string()
        self.model_name      = self.get_parameter("model_name").as_string()
        self.max_retries     = self.get_parameter("max_retries").as_int()
        self.backoff_base    = self.get_parameter("backoff_base").as_double()
        self.connect_timeout = self.get_parameter("connect_timeout").as_int()
        self.read_timeout    = self.get_parameter("read_timeout").as_int()

        # ── Publishers ────────────────────────────────────────────────────────
        self.output_pub_ = self.create_publisher(String, "/llm/raw_output", 10)
        self.status_pub_ = self.create_publisher(String, "/llm/status",     10)

        # ── Subscriber ────────────────────────────────────────────────────────
        self.command_sub_ = self.create_subscription(
            String, "/llm/command_input",
            self.command_callback, 10
        )

        # ── Threading ─────────────────────────────────────────────────────────
        self.executor_pool   = concurrent.futures.ThreadPoolExecutor(max_workers=1)
        self.current_future  = None
        self.current_command = ""
        self.node_state      = "IDLE"

        # ── HTTP session ──────────────────────────────────────────────────────
        self.session = requests.Session()
        self.session.headers.update({"Connection": "keep-alive"})

        # ── 50ms poll timer ───────────────────────────────────────────────────
        self.poll_timer_ = self.create_timer(0.05, self.poll_callback)

        # ── Pre-flight health check ───────────────────────────────────────────
        if not ollama_health_check(self.session, self.ollama_url,
                                    self.connect_timeout):
            self.get_logger().error(
                f"Ollama not reachable at {self.ollama_url}. "
                "Node will start but all commands will fail until Ollama is up."
            )
            self._publish_status("ERROR: Ollama unreachable")
        else:
            self.get_logger().info(
                f"LLM Gateway ready | model={self.model_name} | "
                f"endpoint={self.ollama_url}"
            )
            self._publish_status("READY")

    # ── Subscriber callback ───────────────────────────────────────────────────

    def command_callback(self, msg: String):
        """Receives NL commands. Drops incoming commands if already processing one."""
        command = msg.data.strip()
        if not command:
            return

        if self.node_state != "IDLE":
            self.get_logger().warning(
                f"Gateway BUSY — dropping command: '{command[:60]}'"
            )
            self._publish_status(f"BUSY: dropped '{command[:60]}'")
            return

        self.get_logger().info(f"Command received: '{command}'")
        self.current_command = command
        self.current_future = self.executor_pool.submit(
            query_ollama,
            self.session,
            command,
            self.ollama_url,
            self.model_name,
            self.max_retries,
            self.backoff_base,
            self.connect_timeout,
            self.read_timeout,
            self.get_logger(),
        )
        self.node_state = "WAITING"
        self._publish_status(f"PROCESSING: '{command[:60]}'")

    # ── Poll timer ────────────────────────────────────────────────────────────

    def poll_callback(self):
        """Non-blocking 50ms poll — checks if the inference future is done."""
        if self.node_state != "WAITING":
            return
        if self.current_future is None or not self.current_future.done():
            return

        result = self.current_future.result()
        self.current_future = None
        self.node_state = "IDLE"
        self._handle_result(result)

    # ── Result handler ────────────────────────────────────────────────────────

    def _handle_result(self, result: dict):
        log = self.get_logger()
        cmd = self.current_command

        log.info(
            f"Inference complete | lat={result['latency_ms']}ms | "
            f"attempts={result['attempts']} | syntax={'OK' if result['syntax_ok'] else 'FAIL'}"
        )

        if result["syntax_ok"]:
            msg = String()
            msg.data = result["extracted_payload"]
            self.output_pub_.publish(msg)
            self._publish_status(
                f"OK: {result['extracted_payload']} | {result['latency_ms']}ms"
            )
            log.info(f"Published: {result['extracted_payload']}")
        else:
            self._publish_status(
                f"PARSE_FAIL: '{cmd[:40]}' | {result['error']}"
            )
            log.error(
                f"Failed to parse LLM output for: '{cmd}'\n"
                f"  Raw: {result['raw_output'][:200]}"
            )

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_pub_.publish(msg)

    def destroy_node(self):
        self.executor_pool.shutdown(wait=False)
        self.session.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LLMGatewayNode()

    # MultiThreadedExecutor allows the subscriber callback and poll timer
    # to fire concurrently without blocking each other.
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
