#pragma once
/**
 * @file M3_SerialManipulatorSimulatorFriendly.h
 * @brief A serial-manipulator kinematics model with configurable per-joint
 *        offsets and actuation types, exposed to Python via pybind11.
 *
 * (C) Copyright 2020-2022
 * MIT License
 *
 * Contributors:
 * - Murilo M. Marinho (murilomarinho@ieee.org)
 */


#include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>

namespace DQ_robotics
{

/**
 * @brief A serial manipulator whose joints carry explicit pre/post offsets.
 *
 * Each joint contributes a dual-quaternion transformation of the form
 * @c offset_before_ * actuation(q) * offset_after_, which lets the model
 * represent sensor frames and joint offsets that a plain
 * @ref DQ_SerialManipulator would not. Supports both revolute (R) and
 * prismatic (T) joints about any principal axis.
 */
class M3_SerialManipulatorSimulatorFriendly: public DQ_SerialManipulator
{
public:
    /** @brief The actuation type and axis of a single joint. */
    enum class ActuationType{
        RZ, ///< Revolution about the z-axis.
        RY, ///< Revolution about the y-axis.
        RX, ///< Revolution about the x-axis.
        TZ, ///< Translation along the z-axis.
        TY, ///< Translation along the y-axis.
        TX  ///< Translation along the x-axis.
    };
protected:
    std::vector<DQ> offset_before_;
    std::vector<DQ> offset_after_;
    std::vector<ActuationType> actuation_types_;

    DQ _get_w(const int& ith) const;
    DQ _joint_transformation(const double& q, const int& ith) const;
public:


    M3_SerialManipulatorSimulatorFriendly()=delete;
    /**
     * @brief Construct the manipulator.
     *
     * @param offset_before  Per-joint dual-quaternion offset applied before actuation.
     * @param offset_after   Per-joint dual-quaternion offset applied after actuation.
     * @param actuation_types Per-joint actuation type and axis.
     *
     * @throws std::runtime_error if the three vectors do not have equal size.
     */
    M3_SerialManipulatorSimulatorFriendly(const std::vector<DQ>& offset_before,
                                          const std::vector<DQ>& offset_after,
                                          const std::vector<ActuationType>& actuation_types);

    using DQ_SerialManipulator::raw_pose_jacobian;
    using DQ_SerialManipulator::raw_pose_jacobian_derivative;
    using DQ_SerialManipulator::raw_fkm;

    /**
     * @brief Joint types this model can represent.
     * @return A vector containing @c DQ_JointType::REVOLUTE.
     */
    std::vector<DQ_JointType> get_supported_joint_types() const override;

    /**
     * @brief Raw pose Jacobian of the chain up to a given link.
     * @param q_vec         Joint configuration vector.
     * @param to_ith_link   Index of the terminal link.
     * @return An 8 x (to_ith_link+1) dual-quaternion pose Jacobian.
     */
    MatrixXd raw_pose_jacobian(const VectorXd& q_vec, const int& to_ith_link) const override;
    /**
     * @brief Time derivative of the raw pose Jacobian.
     * @param q             Joint configuration vector.
     * @param q_dot         Joint velocity vector.
     * @param to_ith_link   Index of the terminal link.
     * @return An 8 x (to_ith_link+1) Jacobian-derivative matrix.
     */
    MatrixXd raw_pose_jacobian_derivative(const VectorXd& q, const VectorXd& q_dot, const int& to_ith_link) const override;
    /**
     * @brief Raw forward kinematics of the chain up to a given link.
     * @param q_vec         Joint configuration vector.
     * @param to_ith_link   Index of the terminal link.
     * @return The dual-quaternion pose of the terminal link.
     */
    DQ raw_fkm(const VectorXd &q_vec, const int &to_ith_link) const override;
};

}
