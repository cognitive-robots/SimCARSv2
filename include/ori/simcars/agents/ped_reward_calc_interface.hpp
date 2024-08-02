#pragma once

#include <ori/simcars/geometry/defines.hpp>
#include <ori/simcars/agents/ped_outcome.hpp>
#include <ori/simcars/agents/ped_rewards.hpp>
#include <ori/simcars/agents/ped_reward_parameters.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class IPedRewardCalc
{
public:
    virtual FP_DATA_TYPE calc_reward(PedOutcome const *outcome,
                                     PedRewardParameters const *parameters) const = 0;
    virtual PedRewards calc_rewards(PedOutcome const *outcome,
                                       PedRewardParameters const *parameters) const = 0;
};

}
}
}
