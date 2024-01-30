#pragma once

#include <ori/simcars/geometry/defines.hpp>
#include <ori/simcars/agents/fwd_car_outcome.hpp>
#include <ori/simcars/agents/fwd_car_rewards.hpp>
#include <ori/simcars/agents/fwd_car_reward_parameters.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class IFWDCarRewardCalc
{
public:
    virtual FP_DATA_TYPE calc_reward(FWDCarOutcome const *outcome,
                                     FWDCarRewardParameters const *parameters) const = 0;
    virtual FWDCarRewards calc_rewards(FWDCarOutcome const *outcome,
                                       FWDCarRewardParameters const *parameters) const = 0;
};

}
}
}
