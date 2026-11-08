#pragma once

#include <string>
#include <vector>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include "joint_limits.hpp"


namespace rover_arm
{
    /**
     * IKSolver - Manages the KDL kinematic chain for the rover arm.
     *
     * Responsibilities:
     *     - Parse the URDF into a KDL tree
     *     - Extract the kinematic chain from base_link to tool_tip
     *     - Solve IK for a given cartesian target using Levenberg-Marquardt
     *
     * Decoupled IK: only the first NUM_IK_JOINTS joints (turret, shoulder,
     * elbow) are solved for. The wrist joints are held at their CURRENT angles
     * (from the initial guess) and treated as a rigid extension of the forearm.
     * The operator sets gripper orientation directly with the wrist joints;
     * IK then places the tool tip in XYZ. With 3 joints for 3 position
     * constraints, solutions are unique per elbow branch (no null-space drift).
     *
     * Reach assist: if that 3-joint solve fails (target beyond reach with the
     * wrist where it is, or a joint limit), it retries once with
     * REACH_ASSIST_JOINT (wrist pitch) also free. Near the edge of the
     * workspace this straightens the gripper to extend reach. The wrist pitch
     * then stays where the assist left it.
     *
     * Orientation is not enforced — only position.
     *
     * This is a plain C++ class, not a ROS2 node. It receives the node's
     * logger so it integrates with ROS2 logging without needing a node handle.
     */

    class IKSolver
    {
    public:
        /**
         * Result of an IK solve attempt.
         */
        enum class Result
        {
            SUCCESS,
            NOT_READY,       // chain failed to initialize
            BAD_INPUT,       // wrong number of joints in initial guess
            SOLVER_FAILED,   // LMA solver returned an error
            OUT_OF_REACH,    // solver converged but position error too large
        };

        /**
         * One solved joint. index is the joint's position in the chain, which
         * matches the ArmHAL joint id (ArmHAL::JOINT_NAMES is in chain order).
         */
        struct JointTarget
        {
            unsigned int index;
            double position;  // radians
        };

        /**
         * Constructor.
         * @param urdf_string   Full URDF XML string from robot_description parameter
         * @param limits        Joint limits parsed from URDF, keyed by joint name
         * @param logger        ROS2 logger from the owning node
         */
        IKSolver(
            const std::string & urdf_string,
            const std::unordered_map<std::string, JointLimit> & limits,
            rclcpp::Logger logger
        );

        /** Returns true if the KDL chain was successfully built. */
        bool isReady() const { return ready_; }

        /** Returns the number of joints in the chain, or 0 if not ready. */
        unsigned int getNumJoints() const;

        /** Number of leading chain joints IK solves for (turret, shoulder, elbow). */
        static constexpr unsigned int NUM_IK_JOINTS = 3;

        /**
         * Run FK on given joint positions and return tool tip [x, y, z].
         * Returns false if not ready.
         */
        bool getCurrentPosition(
            const std::vector<double> & joint_positions,
            std::vector<double> & xyz_out
        );

        /**
         * Solve IK for a cartesian target.
         *
         * @param target          Desired tool tip pose (only position is enforced)
         * @param current_joints  Current positions of ALL chain joints (radians).
         *                        Free joints use these as the initial guess;
         *                        all others are held fixed during the solve.
         * @param result          Output: only the joints IK moved (3, or 4 when
         *                        reach assist was used). Callers must command
         *                        exactly these joints and leave the rest alone.
         * @return                Result enum describing success or failure reason
         */
        Result solve(
            const geometry_msgs::msg::PoseStamped & target,
            const std::vector<double> & current_joints,
            std::vector<JointTarget> & result
        );

        /** Human readable string for a Result value, useful for logging. */
        static const char * resultToString(Result r);

    private:
        rclcpp::Logger logger_;
        KDL::Tree tree_;
        KDL::Chain chain_;
        bool ready_ = false;

        KDL::JntArray q_min_;
        KDL::JntArray q_max_;

        // FK over the full chain (base_link -> tool_tip)
        std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;

        // Joint sets (chain joint indices) for the normal and reach-assist solves.
        // assist_joints_ is empty if REACH_ASSIST_JOINT isn't in the chain.
        std::vector<unsigned int> base_joints_;
        std::vector<unsigned int> assist_joints_;

        // Steady clock for throttling warnings (a limit can fail at 20 Hz)
        rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

        /**
         * Build a sub-chain where only joints in `free_joints` can move; every
         * other joint becomes a fixed segment posed at its current angle.
         */
        KDL::Chain buildIkChain(
            const KDL::JntArray & q_full,
            const std::vector<unsigned int> & free_joints) const;

        /**
         * One LMA solve over `free_joints`. On failure, `why` describes it.
         */
        Result solveWith(
            const std::vector<unsigned int> & free_joints,
            const KDL::JntArray & q_full,
            const KDL::Frame & target,
            std::vector<JointTarget> & result,
            std::string & why) const;

        // Position error threshold — if the solver converges but the
        // tool tip is further than this from the target, we reject it
        static constexpr double POSITION_ERROR_THRESHOLD_M = 0.005; // 5mm

        static constexpr const char * ROOT_LINK = "base_link";
        static constexpr const char * TIP_LINK  = "tool_tip";
        static constexpr const char * REACH_ASSIST_JOINT = "wrist_pitch_joint";
    };
} // namespace rover_arm