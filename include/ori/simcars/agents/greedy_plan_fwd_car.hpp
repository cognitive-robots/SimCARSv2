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
#include <ori/simcars/agents/causal/variable_types/endogenous/scalar_goal_val_part.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/scalar_goal_time_part.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/id_goal_val_part.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/id_goal_time_part.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/fwd_car_action_speed_part.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/fwd_car_action_lane_part.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/generate_fwd_car_actions.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/sim_fwd_car_action_outcome.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/calc_fwd_car_action_outcome_reward.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/max_reward_fwd_car_action.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class GreedyPlanFWDCar : public virtual PlanFWDCar
{
protected:
    void init_links() override;

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

    causal::FWDCarRewardParametersFixedVariable reward_params;
    causal::CalcFWDCarActionOutcomeRewardVariable action_outcome_rewards;

    causal::MaxRewardFWDCarActionVariable best_action;

    causal::FWDCarActionSpeedPartVariable best_action_speed_goal;
    causal::ScalarGoalValPartVariable best_action_speed_goal_val;
    causal::ScalarGoalTimePartVariable best_action_speed_goal_time;

    causal::FWDCarActionLanePartVariable best_action_lane_goal;
    causal::IdGoalValPartVariable best_action_lane_goal_val;
    causal::IdGoalTimePartVariable best_action_lane_goal_time;

public:
    GreedyPlanFWDCar(map::IMap const *map, IFWDCarOutcomeSim const *fwd_car_outcome_sim,
                     FWDCarSimParameters fwd_car_sim_parameters,
                     IFWDCarRewardCalc const *fwd_car_reward_calc,
                     FWDCarRewardParameters fwd_car_reward_parameters,
                     FP_DATA_TYPE speed_min_value, FP_DATA_TYPE speed_max_value,
                     FP_DATA_TYPE speed_interval_value, FP_DATA_TYPE time_horizon_value,
                     FP_DATA_TYPE time_interval_value);
};

}
}
}
