#pragma once

#include <ori/simcars/agents/fwd_car_reward_calc_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class DefaultFWDCarRewardCalc : public virtual IFWDCarRewardCalc
{
public:
    FP_DATA_TYPE calc_reward(FWDCarOutcome const *outcome,
                             FWDCarRewardParameters const *parameters) const override;
};

}
}
}
