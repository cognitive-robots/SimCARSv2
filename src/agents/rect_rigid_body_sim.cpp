
#include <ori/simcars/agents/rect_rigid_body_sim.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

RectRigidBodySim::RectRigidBodySim(RectRigidBody *rect_rigid_body,
                                   temporal::Time start_time) :
    PointMass(*rect_rigid_body),
    PointMassSim(rect_rigid_body, start_time),
    RectRigidBody(*rect_rigid_body),

    original_rect_rigid_body(rect_rigid_body),

    sim_dist_headway(rect_rigid_body->get_dist_headway_variable(), &(this->dist_headway_buff),
                     start_time),

    sim_env_torque(rect_rigid_body->get_env_torque_variable(), &(this->env_torque_buff),
                   start_time),
    sim_other_torque(rect_rigid_body->get_other_torque_variable(), &(this->other_torque_buff),
                   start_time),
    sim_ang_acc(rect_rigid_body->get_ang_acc_variable(), &(this->ang_acc_buff), start_time),
    sim_ang_vel(rect_rigid_body->get_ang_vel_variable(), &(this->ang_vel_buff), start_time),
    sim_rot(rect_rigid_body->get_rot_variable(), &(this->rot_buff), start_time)
{
    total_torque.set_endogenous_parent(&sim_env_torque);
    total_torque.set_other_parent(&sim_other_torque);

    prev_ang_acc.set_parent(&sim_ang_acc);

    prev_ang_vel.set_parent(&sim_ang_vel);

    rot_diff.set_parent(&sim_ang_vel);
    prev_rot.set_parent(&sim_rot);

    rect.set_endogenous_parent_1(&sim_pos);
    rect.set_endogenous_parent_2(&sim_rot);

    rot_buff.set_axiomatic(false);
}

temporal::Time RectRigidBodySim::get_min_time() const
{
    return std::min(original_rect_rigid_body->get_min_time(), RectRigidBody::get_min_time());
}

temporal::Time RectRigidBodySim::get_max_time() const
{
    return std::max(original_rect_rigid_body->get_max_time(), RectRigidBody::get_max_time());
}

simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* RectRigidBodySim::get_dist_headway_variable()
{
    return &sim_dist_headway;
}

simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* RectRigidBodySim::get_env_torque_variable()
{
    return &sim_env_torque;
}

simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* RectRigidBodySim::get_other_torque_variable()
{
    return &sim_other_torque;
}

simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* RectRigidBodySim::get_ang_acc_variable()
{
    return &sim_ang_acc;
}

simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* RectRigidBodySim::get_ang_vel_variable()
{
    return &sim_ang_vel;
}

simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* RectRigidBodySim::get_rot_variable()
{
    return &sim_rot;
}

}
}
}
