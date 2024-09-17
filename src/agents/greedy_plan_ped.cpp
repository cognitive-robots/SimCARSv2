
#include <ori/simcars/agents/greedy_plan_ped.hpp>

#include <ori/simcars/agents/ped.hpp>
#include <ori/simcars/agents/control_ped.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

void GreedyPlanPed::init_links()
{
    pos.set_parent(control_ped->get_ped()->get_pos_variable());

    action.set_parent(&best_action);
}

GreedyPlanPed::GreedyPlanPed(map::IPedMap const *map, IPedOutcomeSim const *ped_outcome_sim,
                             PedSimParameters ped_sim_parameters,
                             IPedRewardCalc const *ped_reward_calc,
                             PedRewardParameters ped_reward_parameters,
                             FP_DATA_TYPE time_horizon_value, FP_DATA_TYPE time_interval_value) :
    map(map),
    outcome_sim(ped_outcome_sim),
    reward_calc(ped_reward_calc),

    task(),
    task_proxy(&task),

    time_horizon(time_horizon_value),
    time_horizon_proxy(&time_horizon),
    time_interval(time_interval_value),
    time_options(&time_horizon_proxy, &time_interval),

    pos(),
    node(&pos, map),
    node_options(&node, map),

    actions(&time_options, &node_options),

    sim_params(ped_sim_parameters),
    sim_action_outcomes(&actions, &sim_params, ped_outcome_sim),
    sim_action_outcomes_buff(&sim_action_outcomes),

    reward_params(ped_reward_parameters),
    reward_params_proxy(&reward_params),
    action_outcome_rewards(&sim_action_outcomes_buff, &task_proxy, &reward_params, ped_reward_calc),

    best_outcome_action_pair(&action_outcome_rewards),
    best_action(&best_outcome_action_pair)
{
}

GreedyPlanPed::GreedyPlanPed(GreedyPlanPed const &plan_ped) :
    map(plan_ped.map),
    outcome_sim(plan_ped.outcome_sim),
    reward_calc(plan_ped.reward_calc),

    task(),
    task_proxy(&task),

    time_horizon(plan_ped.time_horizon),
    time_horizon_proxy(&time_horizon),
    time_interval(plan_ped.time_interval),
    time_options(&time_horizon_proxy, &time_interval),

    pos(),
    node(&pos, plan_ped.map),
    node_options(&node, plan_ped.map),

    actions(&time_options, &node_options),

    sim_params(plan_ped.sim_params),
    sim_action_outcomes(&actions, &sim_params, plan_ped.outcome_sim),
    sim_action_outcomes_buff(&sim_action_outcomes),

    reward_params(plan_ped.reward_params),
    reward_params_proxy(&reward_params),
    action_outcome_rewards(&sim_action_outcomes_buff, &task_proxy, &reward_params, plan_ped.reward_calc),

    best_outcome_action_pair(&action_outcome_rewards),
    best_action(&best_outcome_action_pair)
{
}

simcars::causal::IEndogenousVariable<PedRewardParameters>* GreedyPlanPed::get_reward_params_variable()
{
    return &reward_params_proxy;
}

simcars::causal::IEndogenousVariable<PedOutcomeActionPair>* GreedyPlanPed::get_best_outcome_action_pair_variable()
{
    return &best_outcome_action_pair;
}

}
}
}
