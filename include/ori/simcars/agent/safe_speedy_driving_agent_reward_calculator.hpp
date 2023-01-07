#pragma once

#include <ori/simcars/agent/driving_agent_reward_calculator_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class SafeSpeedyDrivingAgentRewardCalculator : public virtual ADrivingAgentRewardCalculator
{
public:
    FP_DATA_TYPE calculate_driving_state_reward(agent::IReadOnlyDrivingAgentState const *state) const override;
};

}
}
}
