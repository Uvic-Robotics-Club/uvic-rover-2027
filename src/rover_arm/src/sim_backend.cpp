#include "sim_backend.hpp"
#include "joint_limits.hpp"
#include <stdexcept>
#include <cmath>
#include <algorithm>

namespace rover_arm
{
    static double trapezoid_fraction(double elapsed, double duration)
    {
        if (duration <= 0.0) return 1.0;
        double t = std::clamp(elapsed / duration, 0.0, 1.0);

        constexpr double ACCEL_FRAC  = 0.25;
        constexpr double CRUISE_FRAC = 0.50;
        constexpr double DECEL_FRAC  = 0.25;
        constexpr double V_PEAK = 1.0 / (CRUISE_FRAC + 0.5 * ACCEL_FRAC + 0.5 * DECEL_FRAC);

        if (t < ACCEL_FRAC) {
            double local = t / ACCEL_FRAC;
            return 0.5 * V_PEAK * ACCEL_FRAC * local * local;
        } else if (t < ACCEL_FRAC + CRUISE_FRAC) {
            double accel_end_pos = 0.5 * V_PEAK * ACCEL_FRAC;
            return accel_end_pos + V_PEAK * (t - ACCEL_FRAC);
        } else {
            double local = (t - (ACCEL_FRAC + CRUISE_FRAC)) / DECEL_FRAC;
            double decel_start_pos = 0.5 * V_PEAK * ACCEL_FRAC + V_PEAK * CRUISE_FRAC;
            return decel_start_pos + V_PEAK * DECEL_FRAC * local * (1.0 - 0.5 * local);
        }
    }

    SimBackend::SimBackend(rclcpp::Node * node)
    :   node_(node),
        joint_positions_(ArmHAL::NUM_JOINTS, 0.0),
        traj_(ArmHAL::NUM_JOINTS),
        last_tick_time_(node->now())
    {   
        limits_ = load_joint_limits(node_);

        RCLCPP_INFO(node_->get_logger(), "Loaded %zu joint limits", limits_.size());
        for (const auto & [name, lim] : limits_)
        {
            RCLCPP_INFO(node_->get_logger(), "  %s: [%.4f, %.4f]", name.c_str(), lim.min, lim.max);
        }

        auto period_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(1.0 / TRAJ_TICK_HZ)
        );
        traj_timer_ = node_->create_wall_timer(
            period_ms,
            std::bind(&SimBackend::trajectory_tick, this)
        );

        RCLCPP_INFO(node_->get_logger(), "SimBackend initialised with %d joints", ArmHAL::NUM_JOINTS);
    }

    void SimBackend::trajectory_tick()
    {
        rclcpp::Time now = node_->now();
        double dt = (now - last_tick_time_).seconds();
        last_tick_time_ = now;

        for (int i = 0; i < ArmHAL::NUM_JOINTS; ++i)
        {
            JointTraj & tj = traj_[i];
            if (!tj.active) continue;

            tj.elapsed_s += dt;
            double frac = trapezoid_fraction(tj.elapsed_s, tj.duration_s);
            joint_positions_[i] = tj.start_pos + frac * (tj.target_pos - tj.start_pos);

            if (tj.elapsed_s >= tj.duration_s)
            {
                joint_positions_[i] = tj.target_pos;
                tj.active = false;
                RCLCPP_DEBUG(node_->get_logger(), "SimBackend: joint %s reached target",
                    ArmHAL::JOINT_NAMES[i].c_str());
            }
        }
    }

    void SimBackend::set_joint_command(int joint_id, double value)
    {
        if (joint_id < 0 || joint_id >= ArmHAL::NUM_JOINTS)
            throw std::out_of_range("joint_id " + std::to_string(joint_id) + " is out of range");

        // Clamp to URDF limits if available
        if (limits_.count(ArmHAL::JOINT_NAMES[joint_id]))
        {
            const auto & lim = limits_[ArmHAL::JOINT_NAMES[joint_id]];
            if (value < lim.min || value > lim.max)
            {
                RCLCPP_WARN(node_->get_logger(),
                    "Joint %s command %.4f clamped to [%.4f, %.4f]",
                    ArmHAL::JOINT_NAMES[joint_id].c_str(), value, lim.min, lim.max);
                value = std::clamp(value, lim.min, lim.max);
            }
        }

        JointTraj & tj   = traj_[joint_id];
        tj.start_pos     = joint_positions_[joint_id];
        tj.target_pos    = value;
        tj.duration_s    = DEFAULT_DURATION;
        tj.elapsed_s     = 0.0;
        tj.active        = (std::abs(value - joint_positions_[joint_id]) > 1e-6);
    }

    void SimBackend::set_all_joints(const std::vector<double>& values)
    {
        if (static_cast<int>(values.size()) != ArmHAL::NUM_JOINTS)
            throw std::invalid_argument(
                "Expected " + std::to_string(ArmHAL::NUM_JOINTS) +
                " joint values, got " + std::to_string(values.size()));

        for (int i = 0; i < ArmHAL::NUM_JOINTS; ++i)
            set_joint_command(i, values[i]);
    }

    void SimBackend::move_joint_relative(int joint_id, double delta)
    {
        if (joint_id < 0 || joint_id >= ArmHAL::NUM_JOINTS)
            throw std::out_of_range("joint_id " + std::to_string(joint_id) + " is out of range");

        // Reads directly from internal state — no feedback lag
        double target = joint_positions_[joint_id] + delta;
        set_joint_command(joint_id, target);
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

    bool SimBackend::is_motion_complete() const
    {
        for (const auto & tj : traj_)
            if (tj.active) return false;
        return true;
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
        std::vector<double> home_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        set_all_joints(home_positions);
    }

} // namespace rover_arm    