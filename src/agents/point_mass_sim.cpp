
#include <ori/simcars/agents/point_mass_sim.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

PointMassSim::PointMassSim(PointMass *point_mass,
                           temporal::Time start_time) :
    PointMass(*point_mass),

    original_point_mass(point_mass),

    sim_env_force(point_mass->get_env_force_variable(), &(this->env_force_buff), start_time),
    sim_other_force(point_mass->get_other_force_variable(), &(this->other_force_buff),
                    start_time),
    sim_lin_acc(point_mass->get_lin_acc_variable(), &(this->lin_acc_buff), start_time),
    sim_lin_vel(point_mass->get_lin_vel_variable(), &(this->lin_vel_buff), start_time),
    sim_pos(point_mass->get_pos_variable(), &(this->pos_buff), start_time),

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

    prev_lin_vel.set_parent(&sim_lin_vel);

    pos_diff.set_parent(&sim_lin_vel);
    prev_pos.set_parent(&sim_pos);

    pos_buff.set_axiomatic(false);
}

temporal::Time PointMassSim::get_min_time() const
{
    return std::min(original_point_mass->get_min_time(), PointMass::get_min_time());
}

temporal::Time PointMassSim::get_max_time() const
{
    return std::max(original_point_mass->get_max_time(), PointMass::get_max_time());
}

simcars::causal::IEndogenousVariable<geometry::Vec>* PointMassSim::get_env_force_variable()
{
    return &sim_env_force;
}

simcars::causal::IEndogenousVariable<geometry::Vec>* PointMassSim::get_other_force_variable()
{
    return &sim_other_force;
}

simcars::causal::IEndogenousVariable<geometry::Vec>* PointMassSim::get_lin_acc_variable()
{
    return &sim_lin_acc;
}

simcars::causal::IEndogenousVariable<geometry::Vec>* PointMassSim::get_lin_vel_variable()
{
    return &sim_lin_vel;
}

simcars::causal::IEndogenousVariable<geometry::Vec>* PointMassSim::get_pos_variable()
{
    return &sim_pos;
}

simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* PointMassSim::get_max_env_force_mag_variable()
{
    return &max_env_force_mag;
}

}
}
}
