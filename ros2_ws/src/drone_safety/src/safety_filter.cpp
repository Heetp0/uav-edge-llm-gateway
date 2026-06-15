#include <memory>
#include <string>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "nlohmann/json.hpp"

using json = nlohmann::json;
using namespace std::chrono_literals;

enum class SafetyState {
    NOMINAL,
    KINEMATIC_HOLD, 
    BLIND_HALT      
};

class SafetyFilterNode : public rclcpp::Node {
public:
    SafetyFilterNode() : Node("safety_filter_node"), current_state_(SafetyState::BLIND_HALT), odom_valid_(false), home_set_(false), home_n_(0.0), home_e_(0.0) {

        this->declare_parameter("max_altitude", 100.0);
        this->declare_parameter("max_range_xy", 40.0);
        this->declare_parameter("watchdog_timeout_sec", 0.5);
        this->declare_parameter("ground_noise_tolerance", 0.5);

        max_altitude_ = this->get_parameter("max_altitude").as_double();
        max_range_xy_ = this->get_parameter("max_range_xy").as_double();
        max_range_xy_sq_ = max_range_xy_ * max_range_xy_;
        watchdog_timeout_sec_ = this->get_parameter("watchdog_timeout_sec").as_double();
        ground_noise_tolerance_ = this->get_parameter("ground_noise_tolerance").as_double();

        mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", rclcpp::QoS(10).best_effort());

        velocity_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", rclcpp::QoS(10).best_effort());

        odometry_subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", rclcpp::QoS(10).best_effort(),
            [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(odom_mutex_);
                current_odom_ = *msg;
                last_odom_time_ = this->get_clock()->now();
                odom_valid_ = true;
                
                if (!home_set_ && std::isfinite(msg->position[0]) && std::isfinite(msg->position[1])) {
                    home_n_ = msg->position[0];
                    home_e_ = msg->position[1];
                    home_set_ = true;
                    RCLCPP_INFO(this->get_logger(), "Home position locked at N:%.2f, E:%.2f", home_n_, home_e_);
                }
            });

        llm_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/llm/raw_output", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                this->validate_and_update(msg);
            });

        engage_blind_halt(); 

        timer_ = this->create_wall_timer(
            50ms, std::bind(&SafetyFilterNode::publish_hardware_stream, this));

        RCLCPP_INFO(this->get_logger(),
            "Deterministic C++ Safety Filter Node (Hardware-Tolerant) Initialized.");
    }

private:
    double max_altitude_;   
    double max_range_xy_;   
    double max_range_xy_sq_;
    double watchdog_timeout_sec_;
    double ground_noise_tolerance_; 

    std::mutex setpoint_mutex_;
    px4_msgs::msg::TrajectorySetpoint current_setpoint_;
    SafetyState current_state_; 

    std::mutex odom_mutex_;
    px4_msgs::msg::VehicleOdometry current_odom_;
    rclcpp::Time last_odom_time_;
    bool odom_valid_;
    
    bool home_set_;
    double home_n_;
    double home_e_;

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr llm_subscription_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr velocity_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    double extract_yaw(const px4_msgs::msg::VehicleOdometry& odom) {
        double q_w = odom.q[0], q_x = odom.q[1], q_y = odom.q[2], q_z = odom.q[3];
        return std::atan2(2.0 * (q_w * q_z + q_x * q_y), 1.0 - 2.0 * (q_y * q_y + q_z * q_z));
    }

    void validate_and_update(const std_msgs::msg::String::SharedPtr msg) {
        
        bool has_odom, has_home;
        rclcpp::Time odom_arrival_time;
        px4_msgs::msg::VehicleOdometry odom;
        double h_n, h_e;
        
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            has_odom = odom_valid_;
            has_home = home_set_;
            odom = current_odom_;
            odom_arrival_time = last_odom_time_;
            h_n = home_n_;
            h_e = home_e_;
        }
        
        if (!has_odom || !has_home) {
            RCLCPP_WARN(this->get_logger(), "Odometry/Home uninitialized. Rejecting command.");
            return;
        }

        if ((this->get_clock()->now() - odom_arrival_time).seconds() > watchdog_timeout_sec_) {
            RCLCPP_ERROR(this->get_logger(),
                "Stale odometry in command path. Forcing BLIND_HALT.");
            engage_blind_halt();
            return; 
        }

        // ── BUG FIX: Hoist Hardware Convergence Validation above JSON parsing ──
        if (!std::isfinite(odom.position[0]) || !std::isfinite(odom.position[1]) || !std::isfinite(odom.position[2]) ||
            !std::isfinite(odom.q[0]) || !std::isfinite(odom.q[1]) || !std::isfinite(odom.q[2]) || !std::isfinite(odom.q[3])) {
            RCLCPP_ERROR(this->get_logger(), "PX4 Estimator lost convergence. Forcing Blind Halt.");
            engage_blind_halt();
            return;
        }

        const float q_nan = std::numeric_limits<float>::quiet_NaN();
        
        try {
            json parsed = json::parse(msg->data);

            if (!parsed.contains("action") || !parsed["action"].is_string() || parsed["action"].get<std::string>() != "goto" ||
                !parsed.contains("x") || !parsed["x"].is_number() ||
                !parsed.contains("y") || !parsed["y"].is_number() ||
                !parsed.contains("z") || !parsed["z"].is_number()) {
                throw std::runtime_error("Malformed JSON structure.");
            }

            double x = parsed["x"];
            double y = parsed["y"];
            double z = parsed["z"];

            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
                throw std::runtime_error("Non-finite coordinate in LLM output");
            }

            double yaw = extract_yaw(odom);
            double cos_yaw = std::cos(yaw);
            double sin_yaw = std::sin(yaw);
            double delta_n = (x * cos_yaw) - (y * sin_yaw);
            double delta_e = (x * sin_yaw) + (y * cos_yaw);

            double target_n = odom.position[0] + delta_n;
            double target_e = odom.position[1] + delta_e;
            
            // ── FIX: Pure Relative Kinematic Z-Axis ──
            // FLU dictates Z is strictly body-relative. Up is +Z, Down is -Z.
            // Since NED Down is positive, we subtract the FLU vector.
            double target_d = odom.position[2] - z; 

            if (target_d < -max_altitude_) {
                throw std::runtime_error("Ceiling breached!");
            }
            if (target_d > 0.0) {
                if (target_d <= ground_noise_tolerance_) {
                    target_d = 0.0;
                } else {
                    throw std::runtime_error("Ground collision predicted!");
                }
            }

            double displacement_n = target_n - h_n;
            double displacement_e = target_e - h_e;
            if ((displacement_n * displacement_n) + (displacement_e * displacement_e) > max_range_xy_sq_) {
                throw std::runtime_error("Cylindrical geofence breached!");
            }

            {
                std::lock_guard<std::mutex> lock(setpoint_mutex_);
                current_setpoint_.position[0] = static_cast<float>(target_n);
                current_setpoint_.position[1] = static_cast<float>(target_e); 
                current_setpoint_.position[2] = static_cast<float>(target_d); 
                
                current_setpoint_.velocity = {q_nan, q_nan, q_nan}; 
                current_setpoint_.acceleration = {q_nan, q_nan, q_nan};
                current_setpoint_.yaw = static_cast<float>(yaw); 
                current_setpoint_.yawspeed = q_nan; 
                
                current_state_ = SafetyState::NOMINAL; 
            }

            RCLCPP_INFO(this->get_logger(),
                "Target set. [N: %.2f, E: %.2f, D: %.2f] | Yaw: %.2f rad",
                target_n, target_e, target_d, yaw);

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(),
                "AI ANOMALY: %s. Locking stream to Kinematic Hold.", e.what());
            // Because of the hoist fix, 'odom' is mathematically guaranteed to be finite here.
            engage_kinematic_hold(odom); 
        }
    }

    void engage_kinematic_hold(const px4_msgs::msg::VehicleOdometry& safe_odom) {
        const float q_nan = std::numeric_limits<float>::quiet_NaN();
        std::lock_guard<std::mutex> lock(setpoint_mutex_);
        
        current_setpoint_.position[0] = safe_odom.position[0];
        current_setpoint_.position[1] = safe_odom.position[1];
        current_setpoint_.position[2] = safe_odom.position[2];
        
        current_setpoint_.velocity = {q_nan, q_nan, q_nan}; 
        current_setpoint_.acceleration = {q_nan, q_nan, q_nan};
        current_setpoint_.yaw = static_cast<float>(extract_yaw(safe_odom));
        current_setpoint_.yawspeed = q_nan;               
        
        current_state_ = SafetyState::KINEMATIC_HOLD;
    }

    void engage_blind_halt() {
        const float q_nan = std::numeric_limits<float>::quiet_NaN();
        std::lock_guard<std::mutex> lock(setpoint_mutex_);
        
        current_setpoint_.position     = {q_nan, q_nan, q_nan};
        current_setpoint_.velocity     = {0.0f, 0.0f, 0.0f}; 
        current_setpoint_.acceleration = {q_nan, q_nan, q_nan};
        current_setpoint_.yaw          = q_nan;
        current_setpoint_.yawspeed     = 0.0f;               
        
        current_state_ = SafetyState::BLIND_HALT;
    }

    void publish_hardware_stream() {
        
        auto now = this->get_clock()->now();
        uint64_t px4_synced_time;
        bool odom_alive = true;

        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            if (!odom_valid_) return; 
            
            if (now >= last_odom_time_ && (now - last_odom_time_).seconds() > watchdog_timeout_sec_) {
                odom_alive = false;
            } else {
                auto safe_delta = (now >= last_odom_time_) ? (now - last_odom_time_).nanoseconds() : 0;
                px4_synced_time = current_odom_.timestamp + static_cast<uint64_t>(safe_delta) / 1000;
            }
        }

        if (!odom_alive) {
            bool requires_emergency_halt = false;
            {
                std::lock_guard<std::mutex> lock(setpoint_mutex_);
                requires_emergency_halt = (current_state_ != SafetyState::BLIND_HALT);
            }
            if (requires_emergency_halt) {
                RCLCPP_ERROR(this->get_logger(), "CRITICAL: Odometry stream lost! Forcing Blind Halt.");
                engage_blind_halt();
            }
            px4_synced_time = now.nanoseconds() / 1000; 
        }

        px4_msgs::msg::TrajectorySetpoint stream_setpoint;
        SafetyState state;
        
        {
            std::lock_guard<std::mutex> lock(setpoint_mutex_);
            stream_setpoint = current_setpoint_; 
            state = current_state_;
        }

        auto mode_msg = px4_msgs::msg::OffboardControlMode();
        mode_msg.timestamp = px4_synced_time;
        
        mode_msg.position = (state == SafetyState::NOMINAL || state == SafetyState::KINEMATIC_HOLD); 
        mode_msg.velocity = (state == SafetyState::BLIND_HALT);  
        mode_msg.acceleration = false;
        mode_msg.attitude = false;
        mode_msg.body_rate = false;
        
        stream_setpoint.timestamp = px4_synced_time;

        mode_publisher_->publish(mode_msg);
        velocity_publisher_->publish(stream_setpoint);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyFilterNode>());
    rclcpp::shutdown();
    return 0;
}
