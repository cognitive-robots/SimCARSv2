#pragma once

#include <ori/simcars/geometry/defines.hpp>

#include <iostream>

namespace ori
{
namespace simcars
{
namespace agents
{

struct FWDCarRewards
{
    FP_DATA_TYPE lane_transitions_reward;
    FP_DATA_TYPE caution_reward;
    FP_DATA_TYPE speed_limit_excess_reward;
    FP_DATA_TYPE max_env_force_mag_reward;

    friend std::ostream& operator<<(std::ostream& output_stream, const FWDCarRewards& rewards)
    {
        return output_stream << "Lane Trans. Reward = " << rewards.lane_transitions_reward << ", " <<
                                "Caution Reward = " << rewards.caution_reward << ", " <<
                                "Speed Limit Excess Reward = " << rewards.speed_limit_excess_reward << ", " <<
                                "Max. Env. Force Mag. Reward = " << rewards.max_env_force_mag_reward;
    }
};

}
}
}
