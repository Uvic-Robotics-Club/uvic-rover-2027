#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "can_backend.hpp"
#include "arm_hal.hpp"

// ─────────────────────────────────────────────────────────────────────────────
// Test fixture
// ─────────────────────────────────────────────────────────────────────────────
class CANBackendTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        node_ = std::make_shared<rclcpp::Node>("can_backend_test_node");
        backend_ = std::make_unique<rover_arm::CANBackend>(node_.get());
    }

    void TearDown() override
    {
        backend_.reset();
        node_.reset();
    }

    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<rover_arm::CANBackend> backend_;
};

// ─────────────────────────────────────────────────────────────────────────────
// 1. SetJointCommand_Throws
// ─────────────────────────────────────────────────────────────────────────────
TEST_F(CANBackendTest, SetJointCommand_Throws)
{
    EXPECT_THROW(
        backend_->set_joint_command(rover_arm::ArmHAL::JOINT_SHOULDER, 0.0),
        std::runtime_error);
}

// ─────────────────────────────────────────────────────────────────────────────
// 2. SetAllJoints_Throws
// ─────────────────────────────────────────────────────────────────────────────
TEST_F(CANBackendTest, SetAllJoints_Throws)
{
    const std::vector<double> commands(rover_arm::ArmHAL::NUM_JOINTS, 0.0);
    EXPECT_THROW(backend_->set_all_joints(commands), std::runtime_error);
}

// ─────────────────────────────────────────────────────────────────────────────
// 3. GetFeedback_Throws
// ─────────────────────────────────────────────────────────────────────────────
TEST_F(CANBackendTest, GetFeedback_Throws)
{
    EXPECT_THROW(backend_->get_feedback(), std::runtime_error);
}

// ─────────────────────────────────────────────────────────────────────────────
// 4. IsHealthy_ReturnsFalse
// ─────────────────────────────────────────────────────────────────────────────
TEST_F(CANBackendTest, IsHealthy_ReturnsFalse)
{
    EXPECT_FALSE(backend_->is_healthy());
}

// ─────────────────────────────────────────────────────────────────────────────
// 5. Home_Throws
// ─────────────────────────────────────────────────────────────────────────────
TEST_F(CANBackendTest, Home_Throws)
{
    EXPECT_THROW(backend_->home(), std::runtime_error);
}

// ─────────────────────────────────────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}