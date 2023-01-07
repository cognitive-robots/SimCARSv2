#pragma once

#include <ori/simcars/agent/driving_agent_reward_calculator_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class ADrivingAgentRewardCalculator : public virtual IDrivingAgentRewardCalculator
{
public:
    FP_DATA_TYPE calculate_state_reward(agent::IReadOnlyEntityState const *state) const override;
};

}
}
}
