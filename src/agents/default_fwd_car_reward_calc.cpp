
#include <ori/simcars/agents/default_fwd_car_reward_calc.hpp>

#include <cmath>

namespace ori
{
namespace simcars
{
namespace agents
{

FP_DATA_TYPE DefaultFWDCarRewardCalc::calc_reward(const FWDCarOutcome *outcome,
                                                  const FWDCarRewardParameters *parameters) const
{
    FP_DATA_TYPE lane_transitions_exp = std::exp(outcome->lane_transitions);
    FP_DATA_TYPE lane_transitions_metric = lane_transitions_exp / (lane_transitions_exp + 1.0);
    FP_DATA_TYPE lane_transitions_weighted_metric = parameters->lane_transitions_weight *
            lane_transitions_metric;

    FP_DATA_TYPE final_speed_metric = 1.0 - std::exp(-4.0 * outcome->final_speed /
                                                     parameters->speed_limit);
    FP_DATA_TYPE final_speed_weighted_metric = parameters->final_speed_weight * final_speed_metric;

    FP_DATA_TYPE max_env_force_mag_exp = std::exp(outcome->max_env_force_mag -
                                                  parameters->env_force_mag_limit);
    FP_DATA_TYPE max_env_force_mag_metric = max_env_force_mag_exp / (max_env_force_mag_exp + 1.0);
    FP_DATA_TYPE max_env_force_mag_weighted_metric = parameters->max_env_force_mag_weight *
            max_env_force_mag_metric;

    return lane_transitions_weighted_metric + final_speed_weighted_metric +
            max_env_force_mag_weighted_metric;
}

}
}
}
