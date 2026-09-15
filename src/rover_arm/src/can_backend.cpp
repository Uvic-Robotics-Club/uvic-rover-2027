#include "can_backend.hpp"
#include <stdexcept>

namespace rover_arm 
{

    CANBackend::CANBackend(rclcpp::Node * node) : node_(node)
    {
        // NOT YET IMPLEMENTED
        RCLCPP_WARN(
            node_->get_logger(),
            "CANBackend: this backend is not yet implemented. "
            "Use SimBackend for development until CAN bridge is ready."
        );
    }

    void CANBackend::set_joint_command(int /*joint_id*/, double /*value*/)
    {
        throw std::runtime_error("CANBackend::set_joint_command — NOT YET IMPLEMENTED");
    }
    
    void CANBackend::set_all_joints(const std::vector<double> & /*values*/)
    {
        throw std::runtime_error("CANBackend::set_all_joints — NOT YET IMPLEMENTED");
    }

    void CANBackend::move_joint_relative(int /*joint_id*/, double /*delta*/)
    {
        throw std::runtime_error("CANBackend::move_joint_relative — NOT YET IMPLEMENTED");
    }    
    
    sensor_msgs::msg::JointState CANBackend::get_feedback()
    {
        throw std::runtime_error("CANBackend::get_feedback — NOT YET IMPLEMENTED");
    }
    
    bool CANBackend::is_healthy()
    {
        // Return false rather than throwing — callers check this without try/catch
        RCLCPP_WARN(node_->get_logger(), "CANBackend::is_healthy — NOT YET IMPLEMENTED");
        return false;
    }
    
    void CANBackend::home()
    {
        throw std::runtime_error("CANBackend::home — NOT YET IMPLEMENTED");
    }
} // namespace rover_arm