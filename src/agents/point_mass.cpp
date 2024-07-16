
#include <ori/simcars/agents/point_mass.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

PointMass::PointMass(uint64_t id_value, FP_DATA_TYPE mass_value) :
    id(id_value),
    id_proxy(&id),

    mass(mass_value),
    mass_proxy(&mass),
    mass_recip(&mass),

    env_force(),
    env_force_buff(&env_force, nullptr, true),
    other_force(),
    other_force_buff(&other_force, nullptr, true),
    total_force(&env_force_buff, &other_force_buff),

    lin_acc(&total_force, &mass_recip),
    lin_acc_buff(&lin_acc, nullptr, false),
    prev_lin_acc(&lin_acc_buff),

    lin_vel_diff(&prev_lin_acc),
    prev_lin_vel(&lin_vel_buff),
    lin_vel(&prev_lin_vel, &lin_vel_diff),
    lin_vel_buff(&lin_vel, nullptr, false),

    pos_diff(&lin_vel_buff),
    prev_pos(&pos_buff),
    pos(&prev_pos, &pos_diff),
    pos_buff(&pos, nullptr, true)
{
    assert(mass_value > 0.0);
}

PointMass::PointMass(PointMass const &point_mass) :
    id(point_mass.id),
    id_proxy(&id),

    mass(point_mass.mass),
    mass_proxy(&mass),
    mass_recip(&mass),

    env_force(),
    env_force_buff(&env_force, nullptr, true),
    other_force(),
    other_force_buff(&other_force, nullptr, true),
    total_force(&env_force_buff, &other_force_buff),

    lin_acc(&total_force, &mass_recip),
    lin_acc_buff(&lin_acc, nullptr, false),
    prev_lin_acc(&lin_acc_buff),

    lin_vel_diff(&prev_lin_acc),
    prev_lin_vel(&lin_vel_buff),
    lin_vel(&prev_lin_vel, &lin_vel_diff),
    lin_vel_buff(&lin_vel, nullptr, false),

    pos_diff(&lin_vel_buff),
    prev_pos(&pos_buff),
    pos(&prev_pos, &pos_diff),
    pos_buff(&pos, nullptr, true)
{}

temporal::Time PointMass::get_min_time() const
{
    return pos_buff.get_min_time();
}

temporal::Time PointMass::get_max_time() const
{
    return pos_buff.get_max_time();
}

simcars::causal::IEndogenousVariable<uint64_t>* PointMass::get_id_variable()
{
    return &id_proxy;
}

simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* PointMass::get_mass_variable()
{
    return &mass_proxy;
}

simcars::causal::IEndogenousVariable<geometry::Vec>* PointMass::get_env_force_variable()
{
    return &env_force_buff;
}

simcars::causal::IEndogenousVariable<geometry::Vec>* PointMass::get_other_force_variable()
{
    return &other_force_buff;
}

simcars::causal::IEndogenousVariable<geometry::Vec>* PointMass::get_lin_acc_variable()
{
    return &lin_acc_buff;
}

simcars::causal::IEndogenousVariable<geometry::Vec>* PointMass::get_lin_vel_variable()
{
    return &lin_vel_buff;
}

simcars::causal::IEndogenousVariable<geometry::Vec>* PointMass::get_pos_variable()
{
    return &pos_buff;
}

}
}
}
