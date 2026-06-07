#include <memory>
#include <string>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "nlohmann/json.hpp"

using json = nlohmann::json;

class SafetyFilterNode : public rclcpp::Node {
public:
    SafetyFilterNode() : Node("safety_filter_node") {

        // PX4 XRCE-DDS requires BEST_EFFORT QoS on the trajectory setpoint topic
        velocity_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", rclcpp::QoS(10).best_effort());

        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/llm/raw_output", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                this->validate_and_route(msg);
            });

        RCLCPP_INFO(this->get_logger(),
            "Deterministic C++ Safety Filter Node (XRCE-DDS / nlohmann) Initialized.");
    }

private:
    // ── Geofence constants ────────────────────────────────────────────────────
    static constexpr double MAX_ALTITUDE = 100.0;   // metres AGL ceiling
    static constexpr double MIN_ALTITUDE =   0.0;   // ground floor
    static constexpr double MAX_RANGE_XY =  40.0;   // lateral boundary radius (m)

    // ─────────────────────────────────────────────────────────────────────────
    void validate_and_route(const std_msgs::msg::String::SharedPtr msg) {

        auto setpoint = px4_msgs::msg::TrajectorySetpoint();
        setpoint.timestamp = this->get_clock()->now().nanoseconds() / 1000; // µs

        // ── Safe defaults for ALL fields before any parsing ───────────────────
        // NAN position  → PX4 ignores position control axis
        // NAN acceleration → PX4 ignores acceleration feed-forward
        // NAN yaw / yawspeed → PX4 maintains current heading
        // Zero velocity  → zero-velocity setpoint (used by hover failsafe)
        setpoint.position     = {NAN, NAN, NAN};
        setpoint.velocity     = {0.0f, 0.0f, 0.0f};
        setpoint.acceleration = {NAN, NAN, NAN};
        setpoint.yaw          = NAN;
        setpoint.yawspeed     = NAN;

        try {
            // ── Parse ─────────────────────────────────────────────────────────
            // json::parse throws json::parse_error on malformed input
            json parsed = json::parse(msg->data);

            if (parsed.value("action", "") != "goto") {
                throw std::runtime_error("Missing or invalid 'action' field");
            }

            double x = parsed.value("x", 0.0);
            double y = parsed.value("y", 0.0);
            double z = parsed.value("z", 0.0);

            // ── Geofence: altitude bounds ─────────────────────────────────────
            if (z < MIN_ALTITUDE) {
                throw std::runtime_error(
                    "Below-ground altitude commanded: z=" + std::to_string(z));
            }
            if (z > MAX_ALTITUDE) {
                throw std::runtime_error(
                    "Ceiling exceeded: z=" + std::to_string(z)
                    + " > " + std::to_string(MAX_ALTITUDE));
            }

            // ── Geofence: lateral bounds ──────────────────────────────────────
            if (std::abs(x) > MAX_RANGE_XY || std::abs(y) > MAX_RANGE_XY) {
                throw std::runtime_error(
                    "Lateral boundary exceeded: x=" + std::to_string(x)
                    + " y=" + std::to_string(y));
            }

            // ── Publish valid setpoint (NED frame: z_NED = –z_UP) ────────────
            setpoint.position[0] = static_cast<float>(x);
            setpoint.position[1] = static_cast<float>(y);
            setpoint.position[2] = static_cast<float>(-z);  // ENU → NED inversion

            velocity_publisher_->publish(setpoint);
            RCLCPP_INFO(this->get_logger(),
                "Vector passed geofence. DDS setpoint → [N: %.2f, E: %.2f, D: %.2f]",
                setpoint.position[0], setpoint.position[1], setpoint.position[2]);

        } catch (const json::parse_error& e) {
            RCLCPP_ERROR(this->get_logger(),
                "JSON parse error (LLM syntactic hallucination): %s", e.what());
            publish_hover(setpoint);

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(),
                "CRITICAL ANOMALY CAUGHT: %s. Engaging hover failsafe.", e.what());
            publish_hover(setpoint);
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Hover failsafe: NAN position + zero velocity tells PX4 to drive all
    // translational velocity to zero while holding the current heading.
    // This is NOT a position hold — it is a zero-velocity command. PX4 will
    // maintain attitude and decelerate to a standstill in place.
    // ─────────────────────────────────────────────────────────────────────────
    void publish_hover(px4_msgs::msg::TrajectorySetpoint& setpoint) {
        setpoint.position     = {NAN, NAN, NAN};
        setpoint.velocity     = {0.0f, 0.0f, 0.0f};
        setpoint.acceleration = {NAN, NAN, NAN};
        setpoint.yaw          = NAN;
        setpoint.yawspeed     = NAN;

        velocity_publisher_->publish(setpoint);
        RCLCPP_WARN(this->get_logger(),
            "Hover failsafe active: commanding zero-velocity setpoint.");
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr velocity_publisher_;
};


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyFilterNode>());
    rclcpp::shutdown();
    return 0;
}
