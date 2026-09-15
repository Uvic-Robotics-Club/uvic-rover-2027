#pragma once

#include "arm_hal.hpp"
#include "joint_limits.hpp"
#include <string>
#include <unordered_map>
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
            void move_joint_relative(int joint_id, double delta) override;
            sensor_msgs::msg::JointState get_feedback() override;
            bool is_motion_complete() const;
            bool is_healthy() override;
            void home() override;

        private:
            void trajectory_tick();  // called by the timer at 50 Hz

            rclcpp::Node * node_; 

            std::vector<double> joint_positions_;   // Current joint positions
            std::vector<double> joint_targets_;     // where joints are going

            // Per-joint trajectory state (trapezoid profile)
            struct JointTraj {
                double start_pos   = 0.0;
                double target_pos  = 0.0;
                double duration_s  = 0.0;  // total move duration
                double elapsed_s   = 0.0;  // time since move started
                bool   active      = false;
            };
            std::vector<JointTraj> traj_;

            rclcpp::TimerBase::SharedPtr traj_timer_;
            rclcpp::Time last_tick_time_;

            std::unordered_map<std::string, JointLimit> limits_;

            static constexpr double TRAJ_TICK_HZ    = 50.0;
            static constexpr double DEFAULT_DURATION = 2.0;  // seconds for full move
    };

} // namespace rover_arm