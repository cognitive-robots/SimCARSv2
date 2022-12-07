
#include <ori/simcars/agent/safe_speedy_driving_agent_reward_calculator.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

FP_DATA_TYPE SafeSpeedyDrivingAgentRewardCalculator::calculate_reward(const IReadOnlyDrivingAgentState *state) const
{
    return 1.0f;
}

}
}
}
