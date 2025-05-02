#pragma once

#include <ori/simcars/causal/variable_types/exogenous/scalar_fixed.hpp>
#include <ori/simcars/causal/variable_types/exogenous/vector_socket.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_proxy.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_buffer.hpp>
#include <ori/simcars/causal/variable_types/endogenous/time_range_calc.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_proxy.hpp>
#include <ori/simcars/causal/variable_types/endogenous/node_closest.hpp>
#include <ori/simcars/causal/variable_types/endogenous/node_adjacent.hpp>
#include <ori/simcars/agents/ped_sim_parameters.hpp>
#include <ori/simcars/agents/ped_reward_parameters.hpp>
#include <ori/simcars/agents/plan_ped.hpp>
#include <ori/simcars/agents/causal/variable_types/exogenous/ped_task_socket.hpp>
#include <ori/simcars/agents/causal/variable_types/exogenous/ped_sim_parameters_fixed.hpp>
#include <ori/simcars/agents/causal/variable_types/exogenous/ped_reward_parameters_fixed.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/ped_task_proxy.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/ped_sim_parameters_proxy.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/ped_reward_parameters_proxy.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/generate_ped_actions.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/ped_outcome_action_pair_action_part.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/reward_ped_outcome_action_tuple_reward_part.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/ped_outcome_action_pairs_buffer.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/max_reward_ped_action.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/calc_ped_action_outcome_reward.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/calc_ped_reward_metrics.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/calc_multi_ped_action_outcome_reward.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/calc_ped_action_outcome.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/sim_multi_ped_action_outcome.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class GreedyPlanPed : public PlanPed
{
protected:
    void init_links() override;

    map::IPedMap const *map;
    agents::IPedOutcomeCalc const *outcome_calc;
    agents::IPedOutcomeSim const *outcome_sim;
    agents::IPedRewardCalc const *reward_calc;

    causal::PedTaskSocketVariable task;
    causal::PedTaskProxyVariable task_proxy;

    simcars::causal::ScalarFixedVariable time_horizon;
    simcars::causal::ScalarProxyVariable time_horizon_proxy;
    simcars::causal::ScalarFixedVariable time_interval;
    simcars::causal::TimeRangeCalc time_options;

    simcars::causal::VectorSocketVariable pos;
    simcars::causal::NodeClosestVariable node;
    simcars::causal::NodeAdjacentVariable node_options;

    causal::GeneratePedActionsVariable actions;

    causal::PedSimParametersFixedVariable sim_params;
    causal::PedSimParametersProxyVariable sim_params_proxy;
    causal::SimMultiPedActionOutcomeVariable sim_action_outcomes;
    causal::PedOutcomeActionPairsBufferVariable sim_action_outcomes_buff;

    causal::PedRewardParametersFixedVariable reward_params;
    causal::PedRewardParametersProxyVariable reward_params_proxy;
    causal::CalcMultiPedActionOutcomeRewardVariable action_outcome_rewards;

    causal::MaxRewardPedActionVariable best_outcome_action_pair;
    causal::PedOutcomeActionPairActionPartVariable best_action;

    causal::CalcPedActionOutcomeVariable real_outcome_action;
    causal::CalcPedActionOutcomeRewardVariable real_outcome_action_reward;
    causal::RewardPedOutcomeActionTupleRewardPartVariable real_reward;
    causal::CalcPedRewardMetricsVariable real_reward_metrics;

public:
    GreedyPlanPed(map::IPedMap const *map, IPedOutcomeSim const *ped_outcome_sim,
                  IPedOutcomeCalc const *ped_outcome_calc, PedSimParameters ped_sim_parameters,
                  IPedRewardCalc const *ped_reward_calc, PedRewardParameters ped_reward_parameters,
                  FP_DATA_TYPE time_horizon_value, FP_DATA_TYPE time_interval_value);
    GreedyPlanPed(GreedyPlanPed const &plan_ped);

    PedTask get_task() const;

    void set_task(PedTask const &task_val);

    simcars::causal::IEndogenousVariable<PedRewardParameters>* get_reward_params_variable();
    simcars::causal::IEndogenousVariable<PedOutcomeActionPair>* get_best_outcome_action_pair_variable();
    simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_real_reward_variable();
    simcars::causal::IEndogenousVariable<PedRewards>* get_real_reward_metrics_variable();
};

}
}
}
