
#include <ori/simcars/agents/goal_force_control_ped.hpp>

#include <ori/simcars/agents/ped.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

void GoalForceControlPed::init_links()
{
    pos.set_parent(ped->get_pos_variable());
    lin_vel.set_parent(ped->get_lin_vel_variable());
    mass.set_parent(ped->get_mass_variable());

    ped_force.set_parent(&actual_goal_force);
}

GoalForceControlPed::GoalForceControlPed(map::IPedMap const *map,
                                         FP_DATA_TYPE max_goal_force_mag_value) :
    map(map),

    goal_pos(&node_val_goal, map),
    pos(),
    neg_pos(&pos),
    pos_diff(&goal_pos, &neg_pos),

    time_diff(&node_time_goal),
    time_diff_secs(&time_diff),
    min_act_horizon_secs(1.0),
    actual_act_horizon_secs(&time_diff_secs, &min_act_horizon_secs),
    actual_act_horizon_secs_recip(&actual_act_horizon_secs),

    goal_lin_vel(&pos_diff, &actual_act_horizon_secs_recip),
    lin_vel(),
    neg_lin_vel(&lin_vel),
    lin_vel_diff(&goal_lin_vel, &neg_lin_vel),
    lin_acc(&lin_vel_diff),

    mass(),
    goal_force(&lin_acc, &mass),

    max_goal_force_mag(max_goal_force_mag_value),
    goal_force_mag(&goal_force),
    actual_goal_force_mag(&goal_force_mag, &max_goal_force_mag),

    goal_force_dir(&goal_force),
    actual_goal_force(&goal_force_dir, &actual_goal_force_mag)
{
}

GoalForceControlPed::GoalForceControlPed(const GoalForceControlPed &control_fwd_car) :
    map(control_fwd_car.map),

    goal_pos(&node_val_goal, map),
    pos(),
    neg_pos(&pos),
    pos_diff(&goal_pos, &neg_pos),

    time_diff(&node_time_goal),
    time_diff_secs(&time_diff),
    min_act_horizon_secs(1.0),
    actual_act_horizon_secs(&time_diff_secs, &min_act_horizon_secs),
    actual_act_horizon_secs_recip(&actual_act_horizon_secs),

    goal_lin_vel(&pos_diff, &actual_act_horizon_secs_recip),
    lin_vel(),
    neg_lin_vel(&lin_vel),
    lin_vel_diff(&goal_lin_vel, &neg_lin_vel),
    lin_acc(&lin_vel_diff),

    mass(),
    goal_force(&lin_acc, &mass),

    max_goal_force_mag(control_fwd_car.max_goal_force_mag),
    goal_force_mag(&goal_force),
    actual_goal_force_mag(&goal_force_mag, &max_goal_force_mag),

    goal_force_dir(&goal_force),
    actual_goal_force(&goal_force_dir, &actual_goal_force_mag)
{
}

simcars::causal::IEndogenousVariable<geometry::Vec>* GoalForceControlPed::get_pos_diff_variable()
{
    return &pos_diff;
}

}
}
}
