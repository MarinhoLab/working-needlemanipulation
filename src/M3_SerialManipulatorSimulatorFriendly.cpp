/**
(C) Copyright 2025 Murilo Marinho (murilomarinho@ieee.org)
*/

#include <M3_SerialManipulatorSimulatorFriendly.h>

namespace DQ_robotics
{

M3_SerialManipulatorSimulatorFriendly::M3_SerialManipulatorSimulatorFriendly(const std::vector<DQ> &offset_before,
                                                                             const std::vector<DQ> &offset_after,
                                                                             const std::vector<ActuationType>& actuation_types):
    DQ_SerialManipulator(actuation_types.size()),
    offset_before_(offset_before),
    offset_after_(offset_after),
    actuation_types_(actuation_types)
{
    if(offset_before_.size() != actuation_types_.size() ||
        offset_after_.size() != actuation_types_.size() )
        throw std::runtime_error("Size issue");

}

DQ M3_SerialManipulatorSimulatorFriendly::_joint_transformation(const double &q, const int &ith) const
{
    const auto& before = offset_before_.at(ith);
    const auto& after = offset_after_.at(ith);

    DQ actuation;

    switch(actuation_types_.at(ith))
    {
    case ActuationType::RZ:
        actuation = cos(q/2.0) + k_*sin(q/2.0);
        break;
    case ActuationType::RY:
        actuation = cos(q/2.0) + j_*sin(q/2.0);
        break;
    case ActuationType::RX:
        actuation = cos(q/2.0) + i_*sin(q/2.0);
        break;
    case ActuationType::TZ:
        actuation = 1 + 0.5*E_*k_*q;
        break;
    case ActuationType::TY:
        actuation = 1 + 0.5*E_*j_*q;
        break;
    case ActuationType::TX:
        actuation = 1 + 0.5*E_*i_*q;
        break;
    }

    return before*actuation*after;
}


DQ M3_SerialManipulatorSimulatorFriendly::_get_w(const int &ith) const
{
    switch(actuation_types_.at(ith))
    {
    case ActuationType::RZ:
        return k_;
        break;
    case ActuationType::RY:
        return j_;
        break;
    case ActuationType::RX:
        return i_;
        break;
    case ActuationType::TZ:
        return E_*k_;
        break;
    case ActuationType::TY:
        return E_*j_;
        break;
    case ActuationType::TX:
        return E_*i_;
    }
    throw std::runtime_error("Invalid actuation");
}

DQ  M3_SerialManipulatorSimulatorFriendly::raw_fkm(const VectorXd& q_vec, const int& to_ith_link) const
{
    _check_q_vec(q_vec);
    _check_to_ith_link(to_ith_link);

    DQ q(1);
    int j = 0;
    for (int i = 0; i < (to_ith_link+1); i++) {
        q = q * _joint_transformation(q_vec(i-j), i);
    }
    return q;
}


MatrixXd M3_SerialManipulatorSimulatorFriendly::raw_pose_jacobian(const VectorXd &q_vec, const int &to_ith_link) const
{
    _check_q_vec(q_vec);
    _check_to_ith_link(to_ith_link);

    MatrixXd J = MatrixXd::Zero(8,to_ith_link+1);
    DQ x_effector = raw_fkm(q_vec,to_ith_link);

    DQ x(1);

    for(int i=0;i<to_ith_link+1;i++)
    {
        DQ w = _get_w(i);
        DQ z = 0.5 * Ad(x * offset_before_.at(i), w);
        x = x*_joint_transformation(q_vec(i), i);
        DQ j = z * x_effector;
        J.col(i)= vec8(j);
    }
    return J;
}

MatrixXd M3_SerialManipulatorSimulatorFriendly::raw_pose_jacobian_derivative(const VectorXd &q, const VectorXd &q_dot, const int &to_ith_link) const
{
    _check_q_vec(q);
    _check_q_vec(q_dot);
    _check_to_ith_link(to_ith_link);

    int n = to_ith_link+1;
    DQ x_effector = raw_fkm(q,to_ith_link);
    MatrixXd J    = raw_pose_jacobian(q,to_ith_link);
    VectorXd vec_x_effector_dot = J*q_dot.head(n);
    DQ x = DQ(1);
    MatrixXd J_dot = MatrixXd::Zero(8,n);
    int jth=0;

    for(int i=0;i<n;i++)
    {
        const DQ w = _get_w(i);
        const DQ z = 0.5*x*w*conj(x);

        VectorXd vec_zdot;
        if(i==0)
        {
            vec_zdot = VectorXd::Zero(8,1);
        }
        else
        {
            vec_zdot = 0.5*(haminus8(w*conj(x)) + hamiplus8(x*w)*C8())*raw_pose_jacobian(q,i-1)*q_dot.head(i);
        }

        J_dot.col(jth) = haminus8(x_effector)*vec_zdot + hamiplus8(z)*vec_x_effector_dot;
        x = x*_joint_transformation(q(jth),i);
        jth = jth+1;
    }

    return J_dot;
}

std::vector<DQ_JointType> M3_SerialManipulatorSimulatorFriendly::get_supported_joint_types() const
{
    return {DQ_JointType::REVOLUTE};
}

}
