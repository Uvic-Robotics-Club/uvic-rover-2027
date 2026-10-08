#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

#include "arm_hal.hpp"
#include "sim_backend.hpp"
#include "can_backend.hpp"

#include <memory>
#include <string>
#include <vector>
#include <chrono>

using namespace std::chrono_literals; // allows "50ms" syntax

namespace rover_arm
{
    /**
     * ArmControlNode - ROS2 node for rover arm joint control
     * 
     * Responsibilities:
     *     - Subscribe to /arm/cmd_joint for incoming joint commands
     *     - Pass commands through the HAL to the active backend (sim or CAN)
     *     - Publish /arm/joint_states feedback at a fixed rate
     *     - Publish /arm/healthy to report motor health status
     * 
     * Node selects the backend at startup via the ROS2 parameter:
     *     - "sim" (default)
     *     - "can"
     * 
     * To launch with a specific backend:
     *     ros2 run rover_arm arm_control_node --ros-args -p backend:=sim
     */

     class ArmControlNode : public rclcpp::Node
     {
        public:
            ArmControlNode() : Node("arm_control_node")
            {
                // ===============================
                // Parameters
                // ===============================

                // Declare parameters with defaults.
                this->declare_parameter<std::string>("backend", "sim");
                this->declare_parameter<double>("publish_rate_hz", 20.0);

                std::string backend_name = this->get_parameter("backend").as_string();
                double publish_rate_hz = this->get_parameter("publish_rate_hz").as_double();

                // ===============================
                // Backend selection
                // ===============================

                if (backend_name == "sim")
                {
                    RCLCPP_INFO(get_logger(), "using SimBackend");
                    hal_ = std::make_unique<SimBackend>(this);
                } else if (backend_name == "can") {
                    RCLCPP_INFO(get_logger(), "using CanBackend (NOT YET IMPLEMENTED)");
                    hal_ = std::make_unique<CANBackend>(this);
                } else {
                    RCLCPP_FATAL(get_logger(), "Unknown backend '%s'. Use 'sim' or 'can'.", backend_name.c_str());
                    throw std::invalid_argument("Unknown backend: " + backend_name);
                }

                // ================================
                // Publishers
                // ================================
                // /arm/joint_states - feedback topic, published at fixed rate
                // QoS depth of 10 means up to 10 messages can be queued
                joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/arm/joint_states", 10);

                // /arm/healthy - simple bool indicating whether all motors are healthy
                health_pub_ = this->create_publisher<std_msgs::msg::Bool>("/arm/healthy", 10);

                // ================================
                // Subscribers
                // ================================

                // /arm/cmd_joint - incoming joint commands from operators OR IK node
                // Operators can command any subset of joints by name
                cmd_joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                    "/arm/cmd_joint", 
                    10,
                    std::bind(&ArmControlNode::on_cmd_joint, this, std::placeholders::_1)
                );

                cmd_joint_relative_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                    "/arm/cmd_joint_relative",
                    10,
                    std::bind(&ArmControlNode::on_cmd_joint_relative, this, std::placeholders::_1)
                );
                
                // ================================
                // Feedback timer
                // ================================

                // Publish feedback at a fixed rate regardless of command frequency
                auto period = std::chrono::duration<double>(1.0 / publish_rate_hz);
                auto period_ms = std::chrono::duration_cast<std::chrono::milliseconds>(period);

                feedback_timer_ = this->create_wall_timer(
                    period_ms, 
                    std::bind(&ArmControlNode::publish_feedback, this)
                );

                RCLCPP_INFO(
                    get_logger(), 
                    "ArmControlNode ready (backend=%s, rate=%.0f Hz)", 
                    backend_name.c_str(), 
                    publish_rate_hz
                );
            }

        private:
            
            // ================================
            // Callback: incoming joint command
            // ================================
            void on_cmd_joint(const sensor_msgs::msg::JointState::SharedPtr msg)
            {
                // Validate the message has at least names and positions
                if (msg->name.empty() || msg->position.empty())
                {
                    RCLCPP_WARN(get_logger(), "Received empty cmd_joint message, ignoring.");
                    return;
                }

                if (msg->name.size() != msg->position.size())
                {
                    RCLCPP_WARN(
                        get_logger(),
                        "cmd_joint name/position size mismatch (%zu names, %zu positions), ignorning.",
                        msg->name.size(),
                        msg->position.size()
                    );
                    return;
                }

                // Match names from the message to joint IDs in the HAL
                // This allows partial commands - e.g., commanding only the shoulder
                for (size_t i = 0; i < msg->name.size(); ++i)
                {
                    int joint_id = get_joint_id_by_name(msg->name[i]);

                    if (joint_id < 0)
                    {
                        RCLCPP_WARN(get_logger(), "Unknown joint name '%s', skipping", msg->name[i].c_str());
                        continue;
                    }

                    try
                    {
                        hal_->set_joint_command(joint_id, msg->position[i]);
                        RCLCPP_DEBUG(
                            get_logger(),
                            "Commanded joint '%s' to %.4f rad",
                            msg->name[i].c_str(),
                            msg->position[i]
                        );
                    } catch (const std::exception & e) {
                        RCLCPP_ERROR(get_logger(), "HAL error on joint '%s': %s", msg->name[i].c_str(), e.what());
                    }
                }
            }

            void on_cmd_joint_relative(const sensor_msgs::msg::JointState::SharedPtr msg)
            {
                if (msg->name.empty() || msg->position.empty())
                {
                    RCLCPP_WARN(get_logger(), "Received empty cmd_joint_relative message, ignoring.");
                    return;
                }

                if (msg->name.size() != msg->position.size())
                {
                    RCLCPP_WARN(get_logger(), "cmd_joint_relative name/position size mismatch, ignoring.");
                    return;
                }

                for (size_t i = 0; i < msg->name.size(); ++i)
                {
                    int joint_id = get_joint_id_by_name(msg->name[i]);
                    if (joint_id < 0)
                    {
                        RCLCPP_WARN(get_logger(), "Unknown joint name '%s', skipping", msg->name[i].c_str());
                        continue;
                    }

                    try
                    {
                        hal_->move_joint_relative(joint_id, msg->position[i]);  // position field holds the delta
                    } catch (const std::exception & e) {
                        RCLCPP_ERROR(get_logger(), "HAL error on joint '%s': %s", msg->name[i].c_str(), e.what());
                    }
                }
            }

            // ================================
            // Timer callback: publish feedback at fixed rate
            // ================================

            void publish_feedback()
            {
                // Publish joint state feedback
                try
                {
                    auto state = hal_->get_feedback();
                    joint_state_pub_->publish(state);
                } catch (const std::exception & e) {
                    RCLCPP_ERROR(get_logger(), "Failed to get HAL feedback: %s", e.what());
                }

                // Publish health status
                auto healthy_msg = std_msgs::msg::Bool();
                healthy_msg.data = hal_->is_healthy();
                health_pub_->publish(healthy_msg);

                if (!healthy_msg.data)
                {
                    RCLCPP_WARN(get_logger(), "Arm health check failed - check motor status");
                }
            }

            // ================================
            // Helper: map joint name to joint ID
            // ================================
            int get_joint_id_by_name(const std::string & name) const
            {
                // Search the JOINT_NAMES list for a matching name and return its index
                // Returns -1 if not found
                const auto & names = ArmHAL::JOINT_NAMES;
                for (size_t i = 0; i < names.size(); ++i)
                {
                    if (names[i] == name)
                    {
                        return static_cast<int>(i);
                    }
                }
                return -1; // not found
            }

            // ================================
            // Member variables
            // ================================

            // HAL point - this is the only place the backend is referenced after init
            std::unique_ptr<ArmHAL> hal_;
            
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr health_pub_;
            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_joint_sub_;
            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_joint_relative_sub_;
            rclcpp::TimerBase::SharedPtr feedback_timer_;
     };
} // namespace rover_arm

// ================================
// Main entry point
// ================================

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    try {
        rclcpp::spin(std::make_shared<rover_arm::ArmControlNode>());
    } catch (const std::exception & e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "ArmControlNode failed: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}