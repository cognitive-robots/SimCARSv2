
#include <ori/simcars/agent/driving_agent_reward_calculator_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

FP_DATA_TYPE ADrivingAgentRewardCalculator::calculate_reward(agent::IReadOnlyEntityState const *state) const
{
    return this->calculate_reward(dynamic_cast<agent::IReadOnlyDrivingAgentState const*>(state));
}

}
}
}
