
#include <ori/simcars/agents/rect_rigid_body_sim.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

RectRigidBodySim::RectRigidBodySim(RectRigidBody *rect_rigid_body,
                                   temporal::Time start_time) :
    RectRigidBody(*rect_rigid_body),

    sim_dist_headway(rect_rigid_body->get_dist_headway_variable(), &(this->dist_headway_buff),
                     start_time),

    sim_env_force(rect_rigid_body->get_env_force_variable(), &(this->env_force_buff), start_time),
    sim_other_force(rect_rigid_body->get_other_force_variable(), &(this->other_force_buff),
                    start_time),
    sim_lin_acc(rect_rigid_body->get_lin_acc_variable(), &(this->lin_acc_buff), start_time),
    sim_lin_vel(rect_rigid_body->get_lin_vel_variable(), &(this->lin_vel_buff), start_time),
    sim_pos(rect_rigid_body->get_pos_variable(), &(this->pos_buff), start_time),

    sim_env_torque(rect_rigid_body->get_env_torque_variable(), &(this->env_torque_buff),
                   start_time),
    sim_other_torque(rect_rigid_body->get_other_torque_variable(), &(this->other_torque_buff),
                   start_time),
    sim_ang_acc(rect_rigid_body->get_ang_acc_variable(), &(this->ang_acc_buff), start_time),
    sim_ang_vel(rect_rigid_body->get_ang_vel_variable(), &(this->ang_vel_buff), start_time),
    sim_rot(rect_rigid_body->get_rot_variable(), &(this->rot_buff), start_time),

    zero_max_env_force_mag(0.0),
    zero_max_env_force_mag_proxy(&zero_max_env_force_mag),

    env_force_mag(&sim_env_force),

    sim_max_env_force_mag(&zero_max_env_force_mag_proxy, &max_env_force_mag, start_time),
    prev_max_env_force_mag(&sim_max_env_force_mag),

    max_env_force_mag(&env_force_mag, &prev_max_env_force_mag)
{
    total_force.set_endogenous_parent(&sim_env_force);
    total_force.set_other_parent(&sim_other_force);

    prev_lin_acc.set_parent(&sim_lin_acc);
    prev_ang_acc.set_parent(&sim_ang_acc);

    prev_lin_vel.set_parent(&sim_lin_vel);
    prev_ang_vel.set_parent(&sim_ang_vel);

    pos_diff.set_parent(&sim_lin_vel);
    prev_pos.set_parent(&sim_pos);
    rot_diff.set_parent(&sim_ang_vel);
    prev_rot.set_parent(&sim_rot);

    rect.set_endogenous_parent_1(&sim_pos);
    rect.set_endogenous_parent_2(&sim_rot);

    pos_buff.set_axiomatic(false);
    rot_buff.set_axiomatic(false);
}

simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* RectRigidBodySim::get_dist_headway_variable()
{
    return &sim_dist_headway;
}

simcars::causal::IEndogenousVariable<geometry::Vec>* RectRigidBodySim::get_env_force_variable()
{
    return &sim_env_force;
}

simcars::causal::IEndogenousVariable<geometry::Vec>* RectRigidBodySim::get_other_force_variable()
{
    return &sim_other_force;
}

simcars::causal::IEndogenousVariable<geometry::Vec>* RectRigidBodySim::get_lin_acc_variable()
{
    return &sim_lin_acc;
}

simcars::causal::IEndogenousVariable<geometry::Vec>* RectRigidBodySim::get_lin_vel_variable()
{
    return &sim_lin_vel;
}

simcars::causal::IEndogenousVariable<geometry::Vec>* RectRigidBodySim::get_pos_variable()
{
    return &sim_pos;
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

simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* RectRigidBodySim::get_max_env_force_mag_variable()
{
    return &max_env_force_mag;
}

}
}
}
