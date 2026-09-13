#pragma once

#include <vector>
#include <string>
#include <sensor_msgs/msg/joint_state.hpp>

namespace rover_arm
{
    /**
     * ArmHAL - Hardware Abstraction Layer interface for the rover arm.
     */

    class ArmHAL
    {
        public:

            // Joint indices, use these constants instead of raw numbers
            // to make code more readable and maintainable
            static constexpr int JOINT_TURRET    = 0;
            static constexpr int JOINT_SHOULDER  = 1;
            static constexpr int JOINT_ELBOW     = 2;
            static constexpr int JOINT_WRIST     = 3;
            static constexpr int JOINT_WRIST_ROLL = 4;
            static constexpr int JOINT_HAND      = 5;
            static constexpr int NUM_JOINTS      = 6;

            // Joint names matching the URDF link names
            // Used when publishing JointState messages
            inline static const std::vector<std::string> JOINT_NAMES = {
                "turret_joint", 
                "shoulder_joint",
                "elbow_joint",
                "wrist_pitch_joint",
                "wrist_roll_joint",
                "hand_joint"
            };

            //Virtual destructor
            virtual ~ArmHAL() = default;

            /**
             * Send a position command to a single joint.
             * @param joint_id  Joint index (use the constants above, e.g. JOINT_SHOULDER)
             * @param value     Target position in radians (or metres for gripper)
             */
            virtual void set_joint_command(int joint_id, double value) = 0;

            // Set all joint commands at once. Vector must be exact number of joints.
            virtual void set_all_joints(const std::vector<double>& values) = 0;

            // Get the current feedback from the arm (joint positions, velocities, etc.)
            virtual sensor_msgs::msg::JointState get_feedback() = 0;

            // Check if the arm is healthy and operational
            virtual bool is_healthy() = 0;

            // Home the arm to its default position
            virtual void home() = 0;
    };
} // namespace rover_arm