#pragma once

#include "arm_hal.hpp"
#include <rclcpp/rclcpp.hpp>

namespace rover_arm
{

    /**
     * CANBackend for interacting with the Raspberry Pi
     * 
     * Architecture:
     *     arm_control -> CANBackend -> TCP socket (probably) -> Pi CAN bridge -> motors
     * 
     * Protocol:
     *     - Pi runs a TCP socket server on a fixed port
     *     - Commands are serialized as JSON or a simple binary protocol
     *     - Feedback is read back from the same socket
     */

     class CANBackend : public ArmHAL
     {
        public:
            explicit CANBackend(rclcpp::Node * node);

            // NOT YET IMPLEMENTED - ALL METHODS THROW UNTIL CAN BRIDGE IS READY
            void set_joint_command(int joint_id, double value) override;
            void set_all_joints(const std::vector<double> & values) override;
            void move_joint_relative(int joint_id, double delta) override;
            sensor_msgs::msg::JointState get_feedback() override;
            bool is_healthy() override;
            void home() override;

        private:
            rclcpp::Node * node_;

            // TODO: TCP socket connection to Pi CAN bridge
            // int socket_fd_;
            // std::string pi_ip_;
            // int pi_port_;
    };
} // namespace rover_arm