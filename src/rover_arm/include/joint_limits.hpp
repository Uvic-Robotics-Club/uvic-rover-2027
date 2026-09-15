#pragma once

#include <string>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>

namespace rover_arm
{
    struct JointLimit {
        double min;
        double max;
    };

    // Reads joint limits from the URDF on /robot_description.
    // Returns a map of joint_name -> JointLimit.
    std::unordered_map<std::string, JointLimit> load_joint_limits(rclcpp::Node * node);

} // namespace rover_arm