#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <thread>

#include "sim_backend.hpp"
#include "arm_hal.hpp"

// ─────────────────────────────────────────────────────────────────────────────
// Test fixture
// Spins up a minimal ROS2 node so SimBackend can call node_->now() and log.
// ─────────────────────────────────────────────────────────────────────────────
class SimBackendTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        node_ = std::make_shared<rclcpp::Node>("sim_backend_test_node");
        backend_ = std::make_unique<rover_arm::SimBackend>(node_.get());
    }

    void TearDown() override
    {
        backend_.reset();
        node_.reset();
    }

    void spin_until_done(double timeout_s = 5.0)
    {
        auto start = node_->now();
        while ((node_->now() - start).seconds() < timeout_s)
        {
            rclcpp::spin_some(node_);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));

            // Exit early once all joints have reached their targets
            auto fb = backend_->get_feedback();
            if (backend_->is_motion_complete()) break;  // needs a new method (see below)
        }
    }

    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<rover_arm::SimBackend> backend_;
};

// ─────────────────────────────────────────────────────────────────────────────
// 1. InitialisesAllJointsToZero
// ─────────────────────────────────────────────────────────────────────────────
TEST_F(SimBackendTest, InitialisesAllJointsToZero)
{
    sensor_msgs::msg::JointState feedback = backend_->get_feedback();

    ASSERT_EQ(static_cast<int>(feedback.position.size()), rover_arm::ArmHAL::NUM_JOINTS);

    for (int i = 0; i < rover_arm::ArmHAL::NUM_JOINTS; ++i)
    {
        EXPECT_DOUBLE_EQ(feedback.position[i], 0.0)
            << "Joint " << i << " was not initialised to 0.0";
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 2. SetJointCommand_UpdatesSingleJoint
// ─────────────────────────────────────────────────────────────────────────────
TEST_F(SimBackendTest, SetJointCommand_UpdatesSingleJoint)
{
    constexpr double TARGET = 1.23;
    backend_->set_joint_command(rover_arm::ArmHAL::JOINT_SHOULDER, TARGET);

    spin_until_done();

    sensor_msgs::msg::JointState feedback = backend_->get_feedback();
    
    // The commanded joint should have changed
    EXPECT_DOUBLE_EQ(feedback.position[rover_arm::ArmHAL::JOINT_SHOULDER], TARGET);

    // All other joints must remain at zero
    for (int i = 0; i < rover_arm::ArmHAL::NUM_JOINTS; ++i)
    {
        if (i == rover_arm::ArmHAL::JOINT_SHOULDER) { continue; }
        EXPECT_DOUBLE_EQ(feedback.position[i], 0.0)
            << "Joint " << i << " changed unexpectedly";
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 3. SetJointCommand_OutOfRange_Throws
// ─────────────────────────────────────────────────────────────────────────────
TEST_F(SimBackendTest, SetJointCommand_OutOfRange_Throws)
{
    EXPECT_THROW(backend_->set_joint_command(-1, 0.0),  std::out_of_range);
    EXPECT_THROW(
        backend_->set_joint_command(rover_arm::ArmHAL::NUM_JOINTS, 0.0),
        std::out_of_range);
}

// ─────────────────────────────────────────────────────────────────────────────
// 4. SetAllJoints_UpdatesAllPositions
// ─────────────────────────────────────────────────────────────────────────────
TEST_F(SimBackendTest, SetAllJoints_UpdatesAllPositions)
{
    const std::vector<double> commands = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
    ASSERT_EQ(static_cast<int>(commands.size()), rover_arm::ArmHAL::NUM_JOINTS);

    backend_->set_all_joints(commands);

    spin_until_done();

    sensor_msgs::msg::JointState feedback = backend_->get_feedback();

    for (int i = 0; i < rover_arm::ArmHAL::NUM_JOINTS; ++i)
    {
        EXPECT_DOUBLE_EQ(feedback.position[i], commands[i])
            << "Joint " << i << " position does not match commanded value";
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 5. SetAllJoints_WrongSize_Throws
// ─────────────────────────────────────────────────────────────────────────────
TEST_F(SimBackendTest, SetAllJoints_WrongSize_Throws)
{
    // Too few
    EXPECT_THROW(
        backend_->set_all_joints({0.0, 0.0, 0.0}),
        std::invalid_argument);

    // Too many
    EXPECT_THROW(
        backend_->set_all_joints({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
        std::invalid_argument);
}

// ─────────────────────────────────────────────────────────────────────────────
// 6. GetFeedback_ReturnsCorrectJointNames
// ─────────────────────────────────────────────────────────────────────────────
TEST_F(SimBackendTest, GetFeedback_ReturnsCorrectJointNames)
{
    sensor_msgs::msg::JointState feedback = backend_->get_feedback();

    ASSERT_EQ(feedback.name.size(), rover_arm::ArmHAL::JOINT_NAMES.size());

    for (int i = 0; i < rover_arm::ArmHAL::NUM_JOINTS; ++i)
    {
        EXPECT_EQ(feedback.name[i], rover_arm::ArmHAL::JOINT_NAMES[i])
            << "Joint name at index " << i << " is incorrect";
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 7. GetFeedback_ReturnsCorrectPositions
// ─────────────────────────────────────────────────────────────────────────────
TEST_F(SimBackendTest, GetFeedback_ReturnsCorrectPositions)
{
    backend_->set_joint_command(rover_arm::ArmHAL::JOINT_TURRET,     0.10);
    backend_->set_joint_command(rover_arm::ArmHAL::JOINT_ELBOW,      0.30);
    backend_->set_joint_command(rover_arm::ArmHAL::JOINT_WRIST_PITCH, 0.50);

    spin_until_done();

    sensor_msgs::msg::JointState feedback = backend_->get_feedback();

    EXPECT_DOUBLE_EQ(feedback.position[rover_arm::ArmHAL::JOINT_TURRET],     0.10);
    EXPECT_DOUBLE_EQ(feedback.position[rover_arm::ArmHAL::JOINT_SHOULDER],   0.00);
    EXPECT_DOUBLE_EQ(feedback.position[rover_arm::ArmHAL::JOINT_ELBOW],      0.30);
    EXPECT_DOUBLE_EQ(feedback.position[rover_arm::ArmHAL::JOINT_WRIST_ROLL],      0.00);
    EXPECT_DOUBLE_EQ(feedback.position[rover_arm::ArmHAL::JOINT_WRIST_PITCH], 0.50);
    EXPECT_DOUBLE_EQ(feedback.position[rover_arm::ArmHAL::JOINT_HAND],       0.00);
}

// ─────────────────────────────────────────────────────────────────────────────
// 8. GetFeedback_VelocityAndEffortAreZero
// ─────────────────────────────────────────────────────────────────────────────
TEST_F(SimBackendTest, GetFeedback_VelocityAndEffortAreZero)
{
    // Command something so the backend is not trivially untouched
    backend_->set_all_joints({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});

    spin_until_done();

    sensor_msgs::msg::JointState feedback = backend_->get_feedback();

    ASSERT_EQ(static_cast<int>(feedback.velocity.size()), rover_arm::ArmHAL::NUM_JOINTS);
    ASSERT_EQ(static_cast<int>(feedback.effort.size()),   rover_arm::ArmHAL::NUM_JOINTS);

    for (int i = 0; i < rover_arm::ArmHAL::NUM_JOINTS; ++i)
    {
        EXPECT_DOUBLE_EQ(feedback.velocity[i], 0.0) << "Velocity at joint " << i << " is non-zero";
        EXPECT_DOUBLE_EQ(feedback.effort[i],   0.0) << "Effort at joint "   << i << " is non-zero";
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 9. IsHealthy_AlwaysTrue
// ─────────────────────────────────────────────────────────────────────────────
TEST_F(SimBackendTest, IsHealthy_AlwaysTrue)
{
    EXPECT_TRUE(backend_->is_healthy());

    // Should still be true after commanding joints
    backend_->set_all_joints({0.1, 0.2, 0.3, 0.4, 0.5, 0.6});
    EXPECT_TRUE(backend_->is_healthy());
}

// ─────────────────────────────────────────────────────────────────────────────
// 10. Home_ResetsAllJointsToZero
// ─────────────────────────────────────────────────────────────────────────────
TEST_F(SimBackendTest, Home_ResetsAllJointsToZero)
{
    // Move all joints away from zero first
    backend_->set_all_joints({1.0, 1.0, 1.0, 1.0, 1.0, 1.0});

    spin_until_done();

    backend_->home();

    spin_until_done();

    sensor_msgs::msg::JointState feedback = backend_->get_feedback();

    ASSERT_EQ(static_cast<int>(feedback.position.size()), rover_arm::ArmHAL::NUM_JOINTS);

    for (int i = 0; i < rover_arm::ArmHAL::NUM_JOINTS; ++i)
    {
        EXPECT_DOUBLE_EQ(feedback.position[i], 0.0)
            << "Joint " << i << " was not reset to 0.0 by home()";
    }
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