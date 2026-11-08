#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

#include "arm_hal.hpp"
#include "sim_backend.hpp"
#include "can_backend.hpp"
#include "joint_limits.hpp"
#include "ik_solver.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>

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
                // IK Solver
                // ================================
                std::string urdf = "";
                if (this->has_parameter("robot_description")) {
                    urdf = this->get_parameter("robot_description").as_string();
                } else {
                    this->declare_parameter<std::string>("robot_description", "");
                    urdf = this->get_parameter("robot_description").as_string();
                }

                // Load joint limits to pass into IK solver
                auto joint_limits = rover_arm::load_joint_limits(this);

                ik_solver_ = std::make_unique<IKSolver>(urdf, joint_limits, get_logger());

                if (!ik_solver_->isReady())
                {
                    RCLCPP_WARN(
                        get_logger(),
                        "IKSolver failed to initialize — IK commands will be unavailable. "
                        "Ensure robot_description is being passed to this node."
                    );
                } else {
                    RCLCPP_INFO(
                        get_logger(),
                        "IKSolver ready — %u joints in chain",
                        ik_solver_->getNumJoints()
                    );
                }

                // /arm/ik_status - publishes IK solve result for operator feedback
                ik_status_pub_ = this->create_publisher<std_msgs::msg::String>("/arm/ik_status", 10);

                // /arm/joint_states feedback — store current positions for IK initial guess
                joint_state_feedback_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                    "/arm/joint_states",
                    10,
                    std::bind(&ArmControlNode::on_joint_state_feedback, this, std::placeholders::_1)
                );

                // /arm/cmd_ik_pose — cartesian IK target from input sources
                cmd_ik_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                    "/arm/cmd_ik_pose",
                    10,
                    std::bind(&ArmControlNode::on_cmd_ik_pose, this, std::placeholders::_1)
                );

                // ================================
                // Publishers
                // ================================
                // /arm/joint_states - feedback topic, published at fixed rate
                // QoS depth of 10 means up to 10 messages can be queued
                joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/arm/joint_states", 10);

                // /arm/healthy - simple bool indicating whether all motors are healthy
                health_pub_ = this->create_publisher<std_msgs::msg::Bool>("/arm/healthy", 10);

                // /arm/end_effector_pose - publishes FK result at the feedback rate
                ee_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/arm/end_effector_pose", 10);

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

                // IK interpolation timer — runs at same rate as feedback
                ik_timer_ = this->create_wall_timer(
                    period_ms,
                    std::bind(&ArmControlNode::ik_interpolation_tick, this)
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

                cancel_ik_goal("cmd_joint");

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

                cancel_ik_goal("cmd_joint_relative");

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

            // A direct joint command overrides any IK goal in progress — otherwise
            // ik_interpolation_tick() keeps re-commanding the joints and fights it
            // (e.g. 'home' sent right after leaving jog mode).
            void cancel_ik_goal(const char * reason)
            {
                if (!ik_goal_active_) return;
                ik_goal_active_ = false;
                ik_consecutive_failures_ = 0;
                RCLCPP_INFO(get_logger(), "IK goal cancelled by %s", reason);
            }

            void on_joint_state_feedback(const sensor_msgs::msg::JointState::SharedPtr msg)
            {
                // Store current joint positions for use as IK initial guess
                current_joint_positions_ = msg->position;
            }

            void on_cmd_ik_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
            {
                ik_goal_ = *msg;
                ik_goal_active_ = true;
                // Each new goal gets its own failure budget
                ik_consecutive_failures_ = 0;
            }

            // ================================
            // Interpolation: For jog mode (terminal/controller)
            // ================================
            void ik_interpolation_tick()
            {
                if (!ik_goal_active_ || !ik_solver_->isReady()) return;

                if (current_joint_positions_.size() != ArmHAL::NUM_JOINTS) return;

                // Get current cartesian position via FK
                std::vector<double> current_pos;
                if (!ik_solver_->getCurrentPosition(current_joint_positions_, current_pos)) return;

                // current_pos is [x, y, z]
                double tx = ik_goal_.pose.position.x;
                double ty = ik_goal_.pose.position.y;
                double tz = ik_goal_.pose.position.z;

                double dx = tx - current_pos[0];
                double dy = ty - current_pos[1];
                double dz = tz - current_pos[2];
                double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

                // If close enough, solve directly for goal
                if (dist <= IK_MAX_STEP_M)
                {
                    if (dist < IK_GOAL_TOLERANCE_M) // already there, done
                    {
                        ik_goal_active_ = false;
                        return;
                    }
                    // Once the final goal is solved and commanded, the goal is done.
                    // The backend drives the joints the rest of the way, so there is
                    // no need to keep re-solving every tick while it settles.
                    if (solve_and_send(ik_goal_))
                        ik_goal_active_ = false;
                    return;
                }

                // Step toward goal by IK_MAX_STEP_M
                double scale = IK_MAX_STEP_M / dist;
                geometry_msgs::msg::PoseStamped step_target = ik_goal_;
                step_target.pose.position.x = current_pos[0] + dx * scale;
                step_target.pose.position.y = current_pos[1] + dy * scale;
                step_target.pose.position.z = current_pos[2] + dz * scale;

                solve_and_send(step_target);
            }

            // Returns true if IK succeeded and the joints were commanded.
            bool solve_and_send(const geometry_msgs::msg::PoseStamped & target)
            {
                auto status_msg = std_msgs::msg::String();
                std::vector<IKSolver::JointTarget> result_joints;
                auto result = ik_solver_->solve(target, current_joint_positions_, result_joints);

                if (result != IKSolver::Result::SUCCESS)
                {
                    ik_consecutive_failures_++;
                    status_msg.data = std::string("IK failed: ") + IKSolver::resultToString(result);
                    ik_status_pub_->publish(status_msg);

                    if (ik_consecutive_failures_ >= IK_MAX_CONSECUTIVE_FAILURES)
                    {
                        RCLCPP_WARN(get_logger(),
                            "IK failed %d consecutive times — deactivating goal.",
                            ik_consecutive_failures_);
                        ik_goal_active_ = false;
                        ik_consecutive_failures_ = 0;
                    }
                    return false;
                }

                ik_consecutive_failures_ = 0;

                // Command ONLY the joints IK moved — the others (wrist, hand) are
                // operator-controlled and may be mid-motion
                for (const auto & jt : result_joints)
                {
                    try {
                        hal_->set_joint_command(static_cast<int>(jt.index), jt.position);
                    } catch (const std::exception & e) {
                        RCLCPP_ERROR(get_logger(), "HAL error on joint %u: %s", jt.index, e.what());
                    }
                }

                status_msg.data = "IK OK";
                ik_status_pub_->publish(status_msg);
                return true;
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

                // Publish current end effector pose for jog seeding
                if (ik_solver_->isReady() && !current_joint_positions_.empty())
                {
                    std::vector<double> xyz;
                    if (ik_solver_->getCurrentPosition(current_joint_positions_, xyz))
                    {
                        geometry_msgs::msg::PoseStamped ee_pose;
                        ee_pose.header.stamp = now();
                        ee_pose.header.frame_id = "base_link";
                        ee_pose.pose.position.x = xyz[0];
                        ee_pose.pose.position.y = xyz[1];
                        ee_pose.pose.position.z = xyz[2];
                        ee_pose.pose.orientation.w = 1.0;
                        ee_pose_pub_->publish(ee_pose);
                    }
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
            std::unique_ptr<ArmHAL> hal_;
            std::unique_ptr<rover_arm::IKSolver> ik_solver_;

            int ik_consecutive_failures_ = 0;
            static constexpr int IK_MAX_CONSECUTIVE_FAILURES = 5;

            std::vector<double> current_joint_positions_;

            geometry_msgs::msg::PoseStamped ik_goal_;
            bool ik_goal_active_ = false;
            static constexpr double IK_MAX_STEP_M = 0.005;       // 5mm per tick at 20Hz
            static constexpr double IK_GOAL_TOLERANCE_M = 0.001; // goal considered reached

            rclcpp::TimerBase::SharedPtr ik_timer_;
            
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cmd_ik_pose_sub_;
            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_feedback_sub_;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ik_status_pub_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr health_pub_;
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_pub_;
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