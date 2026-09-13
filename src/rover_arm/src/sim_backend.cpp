#include "sim_backend.hpp"
#include <stdexcept>

namespace rover_arm
{

    SimBackend::SimBackend(rclcpp::Node * node)
    : node_(node),
      joint_positions_(NUM_JOINTS, 0.0), // Initialize all joint positions to 0
      last_command_time_(node_->now())
    {
        RCLCPP_INFO(node_->get_logger(), "SimBackend initialised with %d joints", ArmHAL::NUM_JOINTS);
    }

    void SimBackend::set_joint_command(int joint_id, double value)
    {
        // Bound check for joint_id
        if (joint_id < 0 || joint_id >= ArmHAL::NUM_JOINTS)
        {
            throw std::out_of_range("joint_id " + std::to_string(joint_id) + " is out of range");
        }

        joint_positions_[joint_id] = value;
        last_command_time_ = node_->now();

        RCLCPP_DEBUG(
            node_->get_logger(),
            "SimBackend: joint %s commanded to %.4f rad",
            ArmHAL::JOINT_NAMES[joint_id].c_str(), value
        );
    }

    void SimBackend::set_all_joints(const std::vector<double>& values)
    {
        if (static_cast<int>(values.size()) != ArmHAL::NUM_JOINTS) {
            throw std::invalid_argument(
            "Expected " + std::to_string(ArmHAL::NUM_JOINTS) +
            " joint values, got " + std::to_string(values.size()));
        }
        
        joint_positions_ = values;
        last_command_time_ = node_->now();
        
        RCLCPP_DEBUG(node_->get_logger(), "SimBackend: all joints commanded");
    }

    sensor_msgs::msg::JointState SimBackend::get_feedback()
    {
        sensor_msgs::msg::JointState state;
        
        // Header timestamp — important for ROS2 tools like rqt and RViz
        state.header.stamp = node_->now();
        state.header.frame_id = "";
        
        state.name     = ArmHAL::JOINT_NAMES;
        state.position = joint_positions_;
        
        // In sim, velocity and effort are zero — real hardware would fill these
        // from motor feedback over CAN
        state.velocity.assign(ArmHAL::NUM_JOINTS, 0.0);
        state.effort.assign(ArmHAL::NUM_JOINTS, 0.0);
        
        return state;
    }

    bool SimBackend::is_healthy()
    {
        // Sim is always healthy
        return true;
    }

    void SimBackend::home()
    {
        RCLCPP_INFO(node_->get_logger(), "SimBackend: moving to home position");

        // All zeros for now
        // determined through simulation testing
        std::vector<double> home_positions = {0.0, 0.0, 0.0, 0.0, 0.0};
        set_all_joints(home_positions);
    }

} // namespace rover_arm    