
#include <ori/simcars/agents/greedy_plan_fwd_car.hpp>

#include <ori/simcars/agents/fwd_car.hpp>
#include <ori/simcars/agents/control_fwd_car.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

void GreedyPlanFWDCar::init_links()
{
    pos.set_parent(control_fwd_car->get_fwd_car()->get_pos_variable());

    action.set_parent(&best_action);
}

GreedyPlanFWDCar::GreedyPlanFWDCar(map::IDrivingMap const *map,
                                   IFWDCarOutcomeSim const *fwd_car_outcome_sim,
                                   FWDCarSimParameters fwd_car_sim_parameters,
                                   IFWDCarRewardCalc const *fwd_car_reward_calc,
                                   FWDCarRewardParameters fwd_car_reward_parameters,
                                   FP_DATA_TYPE speed_min_value, FP_DATA_TYPE speed_max_value,
                                   FP_DATA_TYPE speed_interval_value,
                                   FP_DATA_TYPE time_horizon_value,
                                   FP_DATA_TYPE time_interval_value) :
    map(map),
    outcome_sim(fwd_car_outcome_sim),
    reward_calc(fwd_car_reward_calc),

    time_horizon(time_horizon_value),
    time_horizon_proxy(&time_horizon),
    time_interval(time_interval_value),
    time_options(&time_horizon_proxy, &time_interval),

    speed_min(speed_min_value),
    speed_min_proxy(&speed_min),
    speed_max(speed_max_value),
    speed_max_proxy(&speed_max),
    speed_interval(speed_interval_value),
    speed_options(&speed_min_proxy, &speed_max_proxy, &speed_interval),

    pos(),
    lane_options(&pos, map),

    actions(&time_options, &speed_options, &lane_options),

    sim_params(fwd_car_sim_parameters),
    sim_action_outcomes(&actions, &sim_params, fwd_car_outcome_sim),
    sim_action_outcomes_buff(&sim_action_outcomes),

    reward_params(fwd_car_reward_parameters),
    reward_params_proxy(&reward_params),
    action_outcome_rewards(&sim_action_outcomes_buff, &reward_params, fwd_car_reward_calc),

    best_outcome_action_pair(&action_outcome_rewards),
    best_action(&best_outcome_action_pair)
{
}

GreedyPlanFWDCar::GreedyPlanFWDCar(GreedyPlanFWDCar const &plan_fwd_car) :
    map(plan_fwd_car.map),
    outcome_sim(plan_fwd_car.outcome_sim),
    reward_calc(plan_fwd_car.reward_calc),

    time_horizon(plan_fwd_car.time_horizon),
    time_horizon_proxy(&time_horizon),
    time_interval(plan_fwd_car.time_interval),
    time_options(&time_horizon_proxy, &time_interval),

    speed_min(plan_fwd_car.speed_min),
    speed_min_proxy(&speed_min),
    speed_max(plan_fwd_car.speed_max),
    speed_max_proxy(&speed_max),
    speed_interval(plan_fwd_car.speed_interval),
    speed_options(&speed_min_proxy, &speed_max_proxy, &speed_interval),

    pos(),
    lane_options(&pos, plan_fwd_car.map),

    actions(&time_options, &speed_options, &lane_options),

    sim_params(plan_fwd_car.sim_params),
    sim_action_outcomes(&actions, &sim_params, plan_fwd_car.outcome_sim),
    sim_action_outcomes_buff(&sim_action_outcomes),

    reward_params(plan_fwd_car.reward_params),
    reward_params_proxy(&reward_params),
    action_outcome_rewards(&sim_action_outcomes_buff, &reward_params, plan_fwd_car.reward_calc),

    best_outcome_action_pair(&action_outcome_rewards),
    best_action(&best_outcome_action_pair)
{
}

simcars::causal::IEndogenousVariable<FWDCarRewardParameters>* GreedyPlanFWDCar::get_reward_params_variable()
{
    return &reward_params_proxy;
}

simcars::causal::IEndogenousVariable<FWDCarOutcomeActionPair>* GreedyPlanFWDCar::get_best_outcome_action_pair_variable()
{
    return &best_outcome_action_pair;
}

}
}
}
