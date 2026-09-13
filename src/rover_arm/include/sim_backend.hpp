#pragma once

#include "arm_hal.hpp"
#include <rclcpp/rclcpp.hpp>

namespace rover_arm
{

    /**
     * SimBackend - Simulated backend for the rover arm.
     */
    class SimBackend : public ArmHAL
    {    
        public:
            /**
             * Constructor.
             * @param node  The ROS2 node — used for logging (RCLCPP_INFO etc.)
             */
            explicit SimBackend(rclcpp::Node * node);

            // ArmHal interface implementation
            void set_joint_command(int joint_id, double value) override;
            void set_all_joints(const std::vector<double>& values) override;
            sensor_msgs::msg::JointState get_feedback() override;
            bool is_healthy() override;
            void home() override;

        private:
            rclcpp::Node * node_; 

            // Current joint positions - updated on every command
            // In a real Gazebo backend this would be read from gazebo instead
            std::vector<double> joint_positions_;

            // Timestamp of the last command used in JointState message
            rclcpp::Time last_command_time_;
    };

} // namespace rover_arm