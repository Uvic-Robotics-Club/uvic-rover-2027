#include "joint_limits.hpp"
#include <stdexcept>

// tinyxml2 ships with ROS2
#include <tinyxml2.h>  

namespace rover_arm
{

std::unordered_map<std::string, JointLimit> load_joint_limits(rclcpp::Node * node)
{
    std::unordered_map<std::string, JointLimit> limits;

    // robot_state_publisher puts the URDF here
    if (!node->has_parameter("robot_description"))
        node->declare_parameter<std::string>("robot_description", "");

    std::string urdf = node->get_parameter("robot_description").as_string();

    if (urdf.empty())
    {
        RCLCPP_WARN(node->get_logger(),
            "robot_description is empty — joint limits will use defaults");
        return limits;
    }

    tinyxml2::XMLDocument doc;
    if (doc.Parse(urdf.c_str()) != tinyxml2::XML_SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to parse URDF XML");
        return limits;
    }

    // Walk every <joint> element that has a <limit> child
    auto * root = doc.RootElement();  // <robot>
    for (auto * joint = root->FirstChildElement("joint");
         joint != nullptr;
         joint = joint->NextSiblingElement("joint"))
    {
        const char * name = joint->Attribute("name");
        const char * type = joint->Attribute("type");
        if (!name || !type) continue;

        // Fixed and floating joints have no meaningful limits
        if (std::string(type) == "fixed" || std::string(type) == "floating") continue;

        auto * limit_el = joint->FirstChildElement("limit");
        if (!limit_el) continue;

        JointLimit lim;
        limit_el->QueryDoubleAttribute("lower", &lim.min);
        limit_el->QueryDoubleAttribute("upper", &lim.max);
        limits[name] = lim;

        RCLCPP_DEBUG(node->get_logger(),
            "Loaded limits for %s: [%.3f, %.3f]", name, lim.min, lim.max);
    }

    RCLCPP_INFO(node->get_logger(),
        "Loaded joint limits for %zu joints from URDF", limits.size());

    return limits;
}

} // namespace rover_arm