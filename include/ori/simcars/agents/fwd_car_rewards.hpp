#pragma once

#include <ori/simcars/geometry/defines.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

struct FWDCarRewards
{
    FP_DATA_TYPE lane_transitions_reward;
    FP_DATA_TYPE final_speed_reward;
    FP_DATA_TYPE max_env_force_mag_reward;
};

}
}
}
