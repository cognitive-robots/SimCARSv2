#pragma once

#include <ori/simcars/geometry/defines.hpp>
#include <ori/simcars/agent/entity_state_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class IRewardCalculator
{
public:
    virtual ~IRewardCalculator() = default;

    virtual FP_DATA_TYPE calculate_reward(agent::IReadOnlyEntityState const *state) const = 0;
};

}
}
}
