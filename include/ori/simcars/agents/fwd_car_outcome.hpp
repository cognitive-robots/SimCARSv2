#pragma once

#include <ori/simcars/geometry/defines.hpp>

#include <cstdint>

namespace ori
{
namespace simcars
{
namespace agents
{

struct FWDCarOutcome
{
    FP_DATA_TYPE lane_transitions;
    FP_DATA_TYPE final_speed;
    FP_DATA_TYPE max_env_force_mag;

    friend bool operator ==(FWDCarOutcome const &outcome_1, FWDCarOutcome const &outcome_2)
    {
        return outcome_1.lane_transitions == outcome_2.lane_transitions &&
                outcome_1.final_speed == outcome_2.final_speed &&
                outcome_1.max_env_force_mag == outcome_2.max_env_force_mag;
    }
};

}
}
}
