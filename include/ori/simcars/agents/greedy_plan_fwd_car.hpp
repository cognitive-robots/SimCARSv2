#pragma once

#include <ori/simcars/causal/variable_types/exogenous/scalar_fixed.hpp>
#include <ori/simcars/causal/variable_types/exogenous/vector_socket.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_proxy.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_range_calc.hpp>
#include <ori/simcars/causal/variable_types/endogenous/time_range_calc.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_proxy.hpp>
#include <ori/simcars/causal/variable_types/endogenous/lane_selectable.hpp>
#include <ori/simcars/agents/fwd_car_sim_parameters.hpp>
#include <ori/simcars/agents/fwd_car_reward_parameters.hpp>
#include <ori/simcars/agents/plan_fwd_car.hpp>
#include <ori/simcars/agents/causal/variable_types/exogenous/fwd_car_outcome_parameters_fixed.hpp>
#include <ori/simcars/agents/causal/variable_types/exogenous/fwd_car_reward_parameters_fixed.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/fwd_car_reward_parameters_proxy.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/generate_fwd_car_actions.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/fwd_car_outcome_action_pairs_buffer.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/max_reward_fwd_car_action.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/calc_fwd_car_action_outcome_reward.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/sim_fwd_car_action_outcome.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class GreedyPlanFWDCar : public PlanFWDCar
{
protected:
    void init_links() override;

    map::IMap const *map;
    agents::IFWDCarOutcomeSim const *outcome_sim;
    agents::IFWDCarRewardCalc const *reward_calc;

    simcars::causal::ScalarFixedVariable time_horizon;
    simcars::causal::ScalarProxyVariable time_horizon_proxy;
    simcars::causal::ScalarFixedVariable time_interval;
    simcars::causal::TimeRangeCalc time_options;

    simcars::causal::ScalarFixedVariable speed_min;
    simcars::causal::ScalarProxyVariable speed_min_proxy;
    simcars::causal::ScalarFixedVariable speed_max;
    simcars::causal::ScalarProxyVariable speed_max_proxy;
    simcars::causal::ScalarFixedVariable speed_interval;
    simcars::causal::ScalarRangeCalc speed_options;

    simcars::causal::VectorSocketVariable pos;
    simcars::causal::LaneSelectableVariable lane_options;

    causal::GenerateFWDCarActionsVariable actions;

    causal::FWDCarSimParametersFixedVariable sim_params;
    causal::SimFWDCarActionOutcomeVariable sim_action_outcomes;
    causal::FWDCarOutcomeActionPairsBufferVariable sim_action_outcomes_buff;

    causal::FWDCarRewardParametersFixedVariable reward_params;
    causal::FWDCarRewardParametersProxyVariable reward_params_proxy;
    causal::CalcFWDCarActionOutcomeRewardVariable action_outcome_rewards;

    causal::MaxRewardFWDCarActionVariable best_action;

public:
    GreedyPlanFWDCar(map::IMap const *map, IFWDCarOutcomeSim const *fwd_car_outcome_sim,
                     FWDCarSimParameters fwd_car_sim_parameters,
                     IFWDCarRewardCalc const *fwd_car_reward_calc,
                     FWDCarRewardParameters fwd_car_reward_parameters,
                     FP_DATA_TYPE speed_min_value, FP_DATA_TYPE speed_max_value,
                     FP_DATA_TYPE speed_interval_value, FP_DATA_TYPE time_horizon_value,
                     FP_DATA_TYPE time_interval_value);
    GreedyPlanFWDCar(GreedyPlanFWDCar const &plan_fwd_car);

    simcars::causal::IEndogenousVariable<FWDCarRewardParameters>* get_reward_params_variable();
    simcars::causal::IEndogenousVariable<FWDCarAction>* get_best_action_variable();
};

}
}
}
