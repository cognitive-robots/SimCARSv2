
#include <ori/simcars/agent/safe_speedy_driving_agent_reward_calculator.hpp>

#include <iostream>

namespace ori
{
namespace simcars
{
namespace agent
{

FP_DATA_TYPE SafeSpeedyDrivingAgentRewardCalculator::calculate_reward(IReadOnlyDrivingAgentState const *state) const
{
    FP_DATA_TYPE poe_reward = 1.0f;

    temporal::Duration ttc = state->get_ttc_variable()->get_value();
    FP_DATA_TYPE ttc_reward = 1.0f - std::exp(-ttc.count() / 1e3f);
    poe_reward *= ttc_reward;

    temporal::Duration cumilative_collision_time = state->get_cumilative_collision_time_variable()->get_value();
    FP_DATA_TYPE collision_reward = cumilative_collision_time.count() > 0 ? 0.0f : 1.0f;
    poe_reward *= collision_reward;

    FP_DATA_TYPE aligned_linear_velocity = state->get_aligned_linear_velocity_variable()->get_value();
    FP_DATA_TYPE velocity_reward = 1.0f - 0.5f * std::exp(-std::max(aligned_linear_velocity * 1e2f, 0.0f));
    poe_reward *= velocity_reward;

    return poe_reward;
}

}
}
}
