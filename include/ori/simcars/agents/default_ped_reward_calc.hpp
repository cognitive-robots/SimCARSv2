#pragma once

#include <ori/simcars/map/ped_map_interface.hpp>
#include <ori/simcars/agents/ped_reward_calc_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class DefaultPedRewardCalc : public virtual IPedRewardCalc
{
    map::IPedMap const *map;

public:
    DefaultPedRewardCalc(map::IPedMap const *map);

    FP_DATA_TYPE calc_reward(PedOutcome const *outcome, PedTask const *task,
                             PedRewardParameters const *parameters) const override;
    PedRewards calc_rewards(PedOutcome const *outcome, PedTask const *task,
                            PedRewardParameters const *parameters) const override;
};

}
}
}
