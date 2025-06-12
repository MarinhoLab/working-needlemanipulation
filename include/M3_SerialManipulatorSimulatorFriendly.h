#pragma once
/**
(C) Copyright 2020-2022
MIT Lience

Contributors:
- Murilo M. Marinho (murilomarinho@ieee.org)
*/


#include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>

namespace DQ_robotics
{

class M3_SerialManipulatorSimulatorFriendly: public DQ_SerialManipulator
{
public:
    enum class ActuationType{
        RZ,
        RY,
        RX,
        TZ,
        TY,
        TX
    };
protected:
    std::vector<DQ> offset_before_;
    std::vector<DQ> offset_after_;
    std::vector<ActuationType> actuation_types_;

    DQ _get_w(const int& ith) const;
    DQ _joint_transformation(const double& q, const int& ith) const;
public:


    DQ_SerialManipulatorSimulatorFriendly()=delete;
    DQ_SerialManipulatorSimulatorFriendly(const std::vector<DQ>& offset_before,
                                          const std::vector<DQ>& offset_after,
                                          const std::vector<ActuationType> actuation_types);

    using DQ_SerialManipulator::raw_pose_jacobian;
    using DQ_SerialManipulator::raw_pose_jacobian_derivative;
    using DQ_SerialManipulator::raw_fkm;

    MatrixXd raw_pose_jacobian(const VectorXd& q_vec, const int& to_ith_link) const override;
    MatrixXd raw_pose_jacobian_derivative(const VectorXd& q, const VectorXd& q_dot, const int& to_ith_link) const override;
    DQ raw_fkm(const VectorXd &q_vec, const int &to_ith_link) const override;
};

}
