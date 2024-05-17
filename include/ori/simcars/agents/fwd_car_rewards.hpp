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
    FP_DATA_TYPE dist_headway_reward;
    FP_DATA_TYPE max_speed_reward;
    FP_DATA_TYPE anti_speed_reward;
    FP_DATA_TYPE max_env_force_mag_reward;

    friend std::ostream& operator<<(std::ostream& output_stream, const FWDCarRewards& rewards)
    {
        return output_stream << "Lane Trans. Reward = " << rewards.lane_transitions_reward << ", " <<
                                "Dist. Headway Reward = " << rewards.dist_headway_reward << ", " <<
                                "Max. Speed Reward = " << rewards.max_speed_reward << ", " <<
                                "Anti-Speed Reward = " << rewards.anti_speed_reward << ", " <<
                                "Max. Env. Force Mag. Reward = " << rewards.max_env_force_mag_reward;
    }
};

}
}
}
