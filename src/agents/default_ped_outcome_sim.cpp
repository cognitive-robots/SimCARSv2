
#include <ori/simcars/agents/default_ped_outcome_sim.hpp>

#include <ori/simcars/causal/variable_context.hpp>
#include <ori/simcars/agents/point_mass_sim.hpp>
#include <ori/simcars/agents/ped_sim.hpp>
#include <ori/simcars/agents/goal_force_control_ped_sim.hpp>
#include <ori/simcars/agents/action_intervention_ped.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

DefaultPedOutcomeSim::DefaultPedOutcomeSim(GoalForceControlPed *control_ped,
                                           PointMassEnv *point_mass_env) :
    control_ped(control_ped), point_mass_env(point_mass_env)
{
}

PedOutcome DefaultPedOutcomeSim::sim_outcome(PedAction const *action,
                                             PedSimParameters const *parameters) const
{
    temporal::Time start_time = simcars::causal::VariableContext::get_current_time();
    temporal::Duration time_step_size = simcars::causal::VariableContext::get_time_step_size();

    Ped *ped = control_ped->get_ped();

    PedSim *ped_sim = new PedSim(ped, start_time);
    GoalForceControlPedSim control_ped_sim(control_ped, start_time - time_step_size);
    ActionInterventionPed plan_ped_intervention(*action);
    control_ped_sim.set_ped(ped_sim);
    plan_ped_intervention.set_control_ped(&control_ped_sim);

    point_mass_env->remove_point_mass(ped);
    point_mass_env->add_point_mass(ped_sim);


    temporal::Duration sim_horizon = std::chrono::duration_cast<temporal::Duration>(
                std::chrono::duration<FP_DATA_TYPE>(parameters->sim_horizon_secs));
    simcars::causal::VariableContext::set_current_time(start_time + sim_horizon);


    // TODO: Either utilise the Ped Outcome construction variable or remove it from the codebase
    PedOutcome outcome;
    bool res = ped_sim->get_pos_variable()->get_value(outcome.pos);
    if (!res)
    {
        throw std::runtime_error("Could not get position");
    }
    res = ped_sim->get_min_neighbour_dist_variable()->get_value(outcome.min_neighbour_dist);
    if (!res)
    {
        throw std::runtime_error("Could not get minimum neighbour distance");
    }
    geometry::Vec pos_diff;
    res = control_ped_sim.get_pos_diff_variable()->get_value(pos_diff);
    if (!res)
    {
        throw std::runtime_error("Could not get position difference");
    }
    outcome.action_done = pos_diff.norm() <= parameters->action_done_node_dist_threshold;


    point_mass_env->remove_point_mass(ped_sim);
    point_mass_env->add_point_mass(ped);

    delete ped_sim;

    simcars::causal::VariableContext::set_current_time(start_time);

    return outcome;
}

}
}
}
