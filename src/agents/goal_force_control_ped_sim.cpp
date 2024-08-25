
#include <ori/simcars/agents/goal_force_control_ped_sim.hpp>

#include <ori/simcars/agents/ped_sim.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

GoalForceControlPedSim::GoalForceControlPedSim(GoalForceControlPed *goal_force_control_ped,
                                               temporal::Time start_time) :
    GoalForceControlPed(*goal_force_control_ped),

    original_goal_force_control_ped(goal_force_control_ped),

    sim_action(goal_force_control_ped->get_action_variable(), &(this->action_buff), start_time)
{
    node_goal.set_parent(&sim_action);
}

simcars::causal::IEndogenousVariable<PedAction>* GoalForceControlPedSim::get_action_variable()
{
    return &sim_action;
}

}
}
}
