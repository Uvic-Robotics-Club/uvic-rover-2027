#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <fstream>
#include <random>
#include <sstream>

#include "ik_solver.hpp"
#include "joint_limits.hpp"

using rover_arm::IKSolver;

// ─────────────────────────────────────────────────────────────────────────────
// Test fixture
// Builds an IKSolver from the installed URDF, with limits parsed the same way
// arm_control_node does it.
// ─────────────────────────────────────────────────────────────────────────────
class IKSolverTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        const std::string urdf = load_real_urdf();
        node_ = std::make_shared<rclcpp::Node>("ik_solver_test_node");
        node_->declare_parameter<std::string>("robot_description", urdf);
        auto limits = rover_arm::load_joint_limits(node_.get());
        ik_ = std::make_unique<IKSolver>(urdf, limits, node_->get_logger());
    }

    void TearDown() override
    {
        ik_.reset();
        node_.reset();
    }

    static geometry_msgs::msg::PoseStamped make_target(const std::vector<double> & xyz)
    {
        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = "base_link";
        ps.pose.position.x = xyz[0];
        ps.pose.position.y = xyz[1];
        ps.pose.position.z = xyz[2];
        ps.pose.orientation.w = 1.0;
        return ps;
    }

    // A bent, non-singular pose (matches the calibrated home in config.yaml)
    const std::vector<double> HOME = {0.20944, 0.174533, 0.872665, -0.785398, -0.523599, 0.0};

    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<IKSolver> ik_;

private:
    std::string load_real_urdf()
    {
        std::string pkg_share = ament_index_cpp::get_package_share_directory("rover_arm");
        std::string urdf_path = pkg_share + "/urdf/rover_arm.urdf";

        std::ifstream file(urdf_path);
        if (!file.is_open())
        {
            ADD_FAILURE() << "Could not open URDF at: " << urdf_path
                          << " — did the package install correctly?";
            return "";
        }
        std::stringstream ss;
        ss << file.rdbuf();
        return ss.str();
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// Setup
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(IKSolverTest, ChainBuildsWithAllJoints)
{
    ASSERT_TRUE(ik_->isReady());
    EXPECT_EQ(ik_->getNumJoints(), 6u);
}

TEST_F(IKSolverTest, ForwardKinematicsRejectsWrongSize)
{
    std::vector<double> xyz;
    EXPECT_FALSE(ik_->getCurrentPosition({0.0, 0.0, 0.0}, xyz));
    EXPECT_TRUE(ik_->getCurrentPosition(HOME, xyz));
    EXPECT_EQ(xyz.size(), 3u);
}

TEST_F(IKSolverTest, SolveRejectsWrongSizeInitialGuess)
{
    std::vector<IKSolver::JointTarget> result;
    auto r = ik_->solve(make_target({0.0, -0.5, 0.5}), {0.0, 0.0, 0.0}, result);
    EXPECT_EQ(r, IKSolver::Result::BAD_INPUT);
    EXPECT_TRUE(result.empty());
}

// ─────────────────────────────────────────────────────────────────────────────
// Round trip: FK a random pose, then solve back to it from a nearby guess.
// Should always succeed, stay on the same elbow branch, and only move the
// turret/shoulder/elbow.
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(IKSolverTest, RoundTripFromNearbyGuess)
{
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> u(-1.0, 1.0);

    for (int trial = 0; trial < 200; ++trial)
    {
        std::vector<double> q = {u(rng) * 3.0, u(rng) * 1.4, u(rng) * 1.4,
                                 u(rng) * 3.0, u(rng) * 1.4, u(rng) * 3.0};
        std::vector<double> tip;
        ASSERT_TRUE(ik_->getCurrentPosition(q, tip));

        std::vector<double> guess = q;
        for (unsigned int i = 0; i < IKSolver::NUM_IK_JOINTS; ++i)
            guess[i] += u(rng) * 0.05;

        std::vector<IKSolver::JointTarget> result;
        auto r = ik_->solve(make_target(tip), guess, result);
        ASSERT_EQ(r, IKSolver::Result::SUCCESS) << "trial " << trial;

        // Apply the result and confirm FK lands on the target
        std::vector<double> solved = guess;
        for (const auto & jt : result)
        {
            EXPECT_LT(jt.index, 5u) << "hand_joint should never be moved by IK";
            EXPECT_LT(std::abs(jt.position - guess[jt.index]), 0.3)
                << "joint " << jt.index << " jumped — possible elbow-branch flip";
            solved[jt.index] = jt.position;
        }

        std::vector<double> reached;
        ASSERT_TRUE(ik_->getCurrentPosition(solved, reached));
        const double err = std::hypot(reached[0] - tip[0], reached[1] - tip[1], reached[2] - tip[2]);
        EXPECT_LT(err, 0.005) << "trial " << trial;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Jog: step the target 5 mm at a time along each axis from home, the same way
// arm_control_node interpolates. Every axis should get a useful range of travel
// before hitting a limit or the edge of reach.
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(IKSolverTest, JogFromHomeAlongEachAxis)
{
    constexpr double STEP_M = 0.005;
    constexpr int MIN_STEPS = 20;  // at least 100 mm in every direction

    for (int axis = 0; axis < 3; ++axis)
    {
        for (int sign : {-1, 1})
        {
            std::vector<double> q = HOME;
            int steps = 0;
            for (; steps < 400; ++steps)
            {
                std::vector<double> tip;
                ASSERT_TRUE(ik_->getCurrentPosition(q, tip));
                tip[axis] += sign * STEP_M;

                std::vector<IKSolver::JointTarget> result;
                if (ik_->solve(make_target(tip), q, result) != IKSolver::Result::SUCCESS)
                    break;
                for (const auto & jt : result)
                    q[jt.index] = jt.position;
            }
            EXPECT_GE(steps, MIN_STEPS)
                << "axis " << "xyz"[axis] << (sign > 0 ? "+" : "-")
                << " only managed " << steps << " steps";
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Unreachable target fails cleanly with no joints commanded
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(IKSolverTest, UnreachableTargetFailsWithEmptyResult)
{
    std::vector<IKSolver::JointTarget> result;
    auto r = ik_->solve(make_target({5.0, 5.0, 5.0}), HOME, result);
    EXPECT_NE(r, IKSolver::Result::SUCCESS);
    EXPECT_TRUE(result.empty());
}

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    int ret = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return ret;
}
