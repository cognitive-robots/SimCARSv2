#pragma once

#include <ori/simcars/geometry/defines.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

struct FWDCarRewardParameters
{
    FP_DATA_TYPE speed_limit;
    FP_DATA_TYPE env_force_mag_limit;

    FP_DATA_TYPE lane_transitions_weight;
    FP_DATA_TYPE caution_weight;
    FP_DATA_TYPE speed_limit_excess_weight;
    FP_DATA_TYPE max_env_force_mag_weight;
    FP_DATA_TYPE bias_weight;
};

}
}
}
