#include <memory>
#include <string>
#include <cmath>
#include <algorithm> // Required for std::clamp and std::remove
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

class SafetyFilterNode : public rclcpp::Node {
public:
    SafetyFilterNode() : Node("safety_filter_node") {
        // 1. Initialize Publisher
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

        // 2. Initialize Subscriber using a modern C++ Lambda (Fixes Jazzy template deduction errors)
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/llm/raw_output", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                this->validate_and_route(msg);
            });

        RCLCPP_INFO(this->get_logger(), "Deterministic C++ Safety Filter Node Initialized.");
    }

private:
    void validate_and_route(const std_msgs::msg::String::SharedPtr msg) {
        std::string raw_json = msg->data;
        auto cmd_vel = geometry_msgs::msg::Twist();

        const double MAX_TRANSLATION_VELOCITY = 2.0; 
        const double MAX_VERTICAL_VELOCITY = 1.0;    

        try {
            if (raw_json.find("\"action\"") == std::string::npos || raw_json.find("\"goto\"") == std::string::npos) {
                throw std::runtime_error("Malformed JSON or Missing Action Key Field");
            }

            double x = extract_value(raw_json, "\"x\":");
            double y = extract_value(raw_json, "\"y\":");
            double z = extract_value(raw_json, "\"z\":");

            if (std::abs(z) > 0.0 && std::abs(x) == 0.0 && std::abs(y) == 0.0 && std::abs(z) >= 5.0) {
                throw std::runtime_error("Anomalous Vertical Vector Detected: Sign Inversion Suspected.");
            }

            // Enforce hard geometric boundaries
            cmd_vel.linear.x = std::clamp(x, -MAX_TRANSLATION_VELOCITY, MAX_TRANSLATION_VELOCITY);
            cmd_vel.linear.y = std::clamp(y, -MAX_TRANSLATION_VELOCITY, MAX_TRANSLATION_VELOCITY);
            cmd_vel.linear.z = std::clamp(z, -MAX_VERTICAL_VELOCITY, MAX_VERTICAL_VELOCITY);

            velocity_publisher_->publish(cmd_vel);
            RCLCPP_INFO(this->get_logger(), "Vector Passed Safety Envelope. Published: [X: %.2f, Y: %.2f, Z: %.2f]", 
                        cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.linear.z);

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CRITICAL ANOMALY CAUGHT: %s. Engaging Safe Auto-Hover Mode!", e.what());
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.linear.z = 0.0;
            velocity_publisher_->publish(cmd_vel);
        }
    }

    double extract_value(const std::string& json, const std::string& key) {
        size_t pos = json.find(key);
        if (pos == std::string::npos) return 0.0;
        
        size_t start = pos + key.length();
        size_t end = json.find_first_of(",}", start);
        if (end == std::string::npos) return 0.0; 

        std::string val_str = json.substr(start, end - start);
        
        val_str.erase(std::remove(val_str.begin(), val_str.end(), ' '), val_str.end());
        val_str.erase(std::remove(val_str.begin(), val_str.end(), '"'), val_str.end());
        
        // Added rigorous try-catch block for string-to-double conversion safety
        try {
            return std::stod(val_str);
        } catch (...) {
            return 0.0; 
        }
    }

    // 3. FIXED: Removed the invalid nested ::SharedPtr from the template arguments
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyFilterNode>());
    rclcpp::shutdown();
    return 0;
}