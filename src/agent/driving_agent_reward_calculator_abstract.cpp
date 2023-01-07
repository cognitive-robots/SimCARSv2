
#include <ori/simcars/agent/driving_agent_reward_calculator_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

FP_DATA_TYPE ADrivingAgentRewardCalculator::calculate_state_reward(IReadOnlyEntityState const *state) const
{
    return this->calculate_driving_state_reward(dynamic_cast<IReadOnlyDrivingAgentState const*>(state));
}

}
}
}
