#pragma once

#include <ori/simcars/agent/driving_agent_state_interface.hpp>
#include <ori/simcars/agent/reward_calculator_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class IDrivingAgentRewardCalculator : public virtual IRewardCalculator
{
public:
    virtual FP_DATA_TYPE calculate_reward(agent::IReadOnlyDrivingAgentState const *state) const = 0;
};

}
}
}
