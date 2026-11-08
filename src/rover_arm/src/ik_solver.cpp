#include "ik_solver.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace rover_arm
{

    IKSolver::IKSolver(
        const std::string & urdf_string,
        const std::unordered_map<std::string, JointLimit> & limits,
        rclcpp::Logger logger)
        : logger_(logger.get_child("IKSolver"))
    {
        // Per-solve logs are DEBUG (solve runs at 20 Hz). To see them:
        //   ros2 run ... --ros-args --log-level arm_control_node.IKSolver:=debug

        // ================================
        // Step 1: Parse URDF into KDL tree
        // ================================
        if (urdf_string.empty())
        {
            RCLCPP_ERROR(logger_, "IKSolver: robot_description is empty.");
            return;
        }

        if (!kdl_parser::treeFromString(urdf_string, tree_))
        {
            RCLCPP_ERROR(logger_, "IKSolver: failed to parse URDF into KDL tree.");
            return;
        }

        RCLCPP_INFO(logger_, "IKSolver: KDL tree built — %u segments",
            tree_.getNrOfSegments());

        // ================================
        // Step 2: Extract kinematic chain
        // ================================
        if (!tree_.getChain(ROOT_LINK, TIP_LINK, chain_))
        {
            RCLCPP_ERROR(logger_,
                "IKSolver: failed to extract chain from '%s' to '%s'.",
                ROOT_LINK, TIP_LINK);
            return;
        }

        RCLCPP_INFO(logger_, "IKSolver: chain extracted — %u joints, %u segments",
            chain_.getNrOfJoints(), chain_.getNrOfSegments());

        // ================================
        // Step 3: Build joint limit arrays
        // ================================
        const unsigned int n = chain_.getNrOfJoints();
        q_min_.resize(n);
        q_max_.resize(n);

        unsigned int joint_idx = 0;
        for (unsigned int i = 0; i < chain_.getNrOfSegments(); ++i)
        {
            const KDL::Joint & joint = chain_.getSegment(i).getJoint();
            if (joint.getType() == KDL::Joint::None) continue;

            const std::string & name = joint.getName();
            auto it = limits.find(name);

            if (it != limits.end())
            {
                q_min_(joint_idx) = it->second.min;
                q_max_(joint_idx) = it->second.max;
                RCLCPP_INFO(logger_,
                    "IKSolver: joint %u '%s' limits [%.4f, %.4f]",
                    joint_idx, name.c_str(),
                    q_min_(joint_idx), q_max_(joint_idx));
            }
            else
            {
                // Fallback if a joint is missing from the limits map
                q_min_(joint_idx) = -M_PI;
                q_max_(joint_idx) =  M_PI;
                RCLCPP_WARN(logger_,
                    "IKSolver: no limits found for joint '%s', using [-π, π]",
                    name.c_str());
            }

            joint_idx++;
        }

        // ================================
        // Step 4: Create solvers
        // ================================
        fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);

        // ================================
        // Step 5: Joint sets for normal and reach-assist solves
        // ================================
        if (n < NUM_IK_JOINTS)
        {
            RCLCPP_ERROR(logger_,
                "IKSolver: chain has %u joints, need at least %u for IK.", n, NUM_IK_JOINTS);
            return;
        }

        std::vector<std::string> joint_names;
        for (unsigned int i = 0; i < chain_.getNrOfSegments(); ++i)
        {
            const KDL::Joint & j = chain_.getSegment(i).getJoint();
            if (j.getType() != KDL::Joint::None) joint_names.push_back(j.getName());
        }

        std::string ik_names;
        for (unsigned int i = 0; i < NUM_IK_JOINTS; ++i)
        {
            base_joints_.push_back(i);
            ik_names += (i ? ", " : "") + joint_names[i];
        }

        auto assist = std::find(joint_names.begin(), joint_names.end(), REACH_ASSIST_JOINT);
        if (assist != joint_names.end())
        {
            assist_joints_ = base_joints_;
            assist_joints_.push_back(static_cast<unsigned int>(assist - joint_names.begin()));
        }
        else
        {
            RCLCPP_WARN(logger_, "IKSolver: '%s' not in chain — reach assist disabled.",
                REACH_ASSIST_JOINT);
        }

        ready_ = true;
        RCLCPP_INFO(logger_,
            "IKSolver: ready — tip '%s', solving joints [%s], reach assist: %s.",
            TIP_LINK, ik_names.c_str(),
            assist_joints_.empty() ? "off" : REACH_ASSIST_JOINT);
    }

    KDL::Chain IKSolver::buildIkChain(
        const KDL::JntArray & q_full,
        const std::vector<unsigned int> & free_joints) const
    {
        KDL::Chain ik_chain;
        unsigned int q_idx = 0;  // index of the next movable joint in chain_

        for (unsigned int i = 0; i < chain_.getNrOfSegments(); ++i)
        {
            const KDL::Segment & seg = chain_.getSegment(i);

            if (seg.getJoint().getType() == KDL::Joint::None)
            {
                ik_chain.addSegment(seg);  // already fixed
                continue;
            }

            const bool is_free =
                std::find(free_joints.begin(), free_joints.end(), q_idx) != free_joints.end();

            if (is_free)
                ik_chain.addSegment(seg);
            else
                // Lock this joint at its current angle: same transform, no DOF
                ik_chain.addSegment(KDL::Segment(
                    seg.getName(), KDL::Joint(KDL::Joint::None), seg.pose(q_full(q_idx))));

            ++q_idx;
        }
        return ik_chain;
    }

    unsigned int IKSolver::getNumJoints() const
    {
        if (!ready_) return 0;
        return chain_.getNrOfJoints();
    }

    bool IKSolver::getCurrentPosition(
        const std::vector<double> & joint_positions,
        std::vector<double> & xyz_out)
    {
        if (!ready_) return false;

        KDL::JntArray q(chain_.getNrOfJoints());
        for (unsigned int i = 0; i < chain_.getNrOfJoints(); ++i)
            q(i) = joint_positions[i];

        KDL::Frame frame;
        fk_solver_->JntToCart(q, frame);

        xyz_out = {frame.p.x(), frame.p.y(), frame.p.z()};
        return true;
    }

    IKSolver::Result IKSolver::solve(
        const geometry_msgs::msg::PoseStamped & target,
        const std::vector<double> & current_joints,
        std::vector<JointTarget> & result)
    {
        if (!ready_) return Result::NOT_READY;

        const unsigned int n = chain_.getNrOfJoints();

        if (current_joints.size() != n)
        {
            RCLCPP_ERROR(logger_,
                "IKSolver: initial guess has %zu joints, expected %u",
                current_joints.size(), n);
            return Result::BAD_INPUT;
        }

        // Full current joint state — initial guess for free joints, and the
        // angles locked joints are held at
        KDL::JntArray q_full(n);
        for (unsigned int i = 0; i < n; ++i)
            q_full(i) = current_joints[i];

        // Orientation is ignored (zero LMA weights), only position matters
        const KDL::Frame target_frame(
            KDL::Rotation::Identity(),
            KDL::Vector(target.pose.position.x, target.pose.position.y, target.pose.position.z));

        KDL::Frame fk_init;
        fk_solver_->JntToCart(q_full, fk_init);
        RCLCPP_DEBUG(logger_,
            "IKSolver: tip (%.4f, %.4f, %.4f) → target (%.4f, %.4f, %.4f)",
            fk_init.p.x(), fk_init.p.y(), fk_init.p.z(),
            target_frame.p.x(), target_frame.p.y(), target_frame.p.z());

        // 1) Normal: turret, shoulder, elbow — wrist held where it is
        std::string why;
        Result r = solveWith(base_joints_, q_full, target_frame, result, why);
        if (r == Result::SUCCESS) return r;

        // 2) Reach assist: also let wrist pitch move
        if (!assist_joints_.empty())
        {
            std::string assist_why;
            if (solveWith(assist_joints_, q_full, target_frame, result, assist_why) == Result::SUCCESS)
            {
                RCLCPP_DEBUG(logger_, "IKSolver: reach assist used (%s)", why.c_str());
                return Result::SUCCESS;
            }
            why += "; with reach assist: " + assist_why;
        }

        // Pushing against a limit fails every tick — keep the log readable
        RCLCPP_WARN_THROTTLE(logger_, steady_clock_, 1000,
            "IKSolver: no solution for target (%.3f, %.3f, %.3f), %.1f mm from tip — %s",
            target_frame.p.x(), target_frame.p.y(), target_frame.p.z(),
            (target_frame.p - fk_init.p).Norm() * 1000.0, why.c_str());

        result.clear();
        return r;
    }

    IKSolver::Result IKSolver::solveWith(
        const std::vector<unsigned int> & free_joints,
        const KDL::JntArray & q_full,
        const KDL::Frame & target,
        std::vector<JointTarget> & result,
        std::string & why) const
    {
        // Rebuilt every solve: locked joints may have moved since last time
        const KDL::Chain ik_chain = buildIkChain(q_full, free_joints);
        const unsigned int k = free_joints.size();

        KDL::JntArray q_init(k);
        for (unsigned int i = 0; i < k; ++i)
            q_init(i) = q_full(free_joints[i]);

        // LMA with position-only weights
        Eigen::Matrix<double, 6, 1> weights;
        weights << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0;
        KDL::ChainIkSolverPos_LMA ik_solver(
            ik_chain,
            weights,
            1e-5,   // task space convergence tolerance (m)
            500,    // max iterations — 3-4 DOF converges in a handful
            1e-15   // minimum joint increment before giving up
        );

        KDL::JntArray q_out(k);
        const int ret = ik_solver.CartToJnt(q_init, target, q_out);
        if (ret < 0)
        {
            // -100/-101: gradient/increment too small — target unreachable from here
            // -5: max iterations
            why = "solver code " + std::to_string(ret) +
                  (ret == -100 || ret == -101 ? " (beyond reach)" : "");
            return Result::SOLVER_FAILED;
        }

        // Verify with FK on the same sub-chain the solver used
        KDL::Frame fk_frame;
        KDL::ChainFkSolverPos_recursive(ik_chain).JntToCart(q_out, fk_frame);
        const double position_error = (fk_frame.p - target.p).Norm();
        if (position_error > POSITION_ERROR_THRESHOLD_M)
        {
            why = "position error " + std::to_string(position_error * 1000.0) + " mm";
            return Result::OUT_OF_REACH;
        }

        // Reject (don't clamp) out-of-limit solutions — clamping changes the pose
        for (unsigned int i = 0; i < k; ++i)
        {
            const unsigned int j = free_joints[i];
            if (q_out(i) < q_min_(j) - 1e-6 || q_out(i) > q_max_(j) + 1e-6)
            {
                char buf[128];
                std::snprintf(buf, sizeof(buf), "joint %u would be %.3f, limits [%.3f, %.3f]",
                    j, q_out(i), q_min_(j), q_max_(j));
                why = buf;
                return Result::OUT_OF_REACH;
            }
        }

        result.clear();
        for (unsigned int i = 0; i < k; ++i)
            result.push_back({free_joints[i], q_out(i)});
        return Result::SUCCESS;
    }

    const char * IKSolver::resultToString(Result r)
    {
        switch (r)
        {
            case Result::SUCCESS:       return "SUCCESS";
            case Result::NOT_READY:     return "NOT_READY";
            case Result::BAD_INPUT:     return "BAD_INPUT";
            case Result::SOLVER_FAILED: return "SOLVER_FAILED";
            case Result::OUT_OF_REACH:  return "OUT_OF_REACH";
            default:                    return "UNKNOWN";
        }
    }

} // namespace rover_arm