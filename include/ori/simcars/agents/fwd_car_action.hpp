#pragma once

#include <ori/simcars/geometry/defines.hpp>
#include <ori/simcars/agents/goal.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

struct FWDCarAction
{
    Goal<FP_DATA_TYPE> speed_goal;
    Goal<uint64_t> lane_goal;
};

bool operator ==(FWDCarAction const &action_1, FWDCarAction const &action_2)
{
    return action_1.speed_goal == action_2.speed_goal && action_1.lane_goal == action_2.lane_goal;
}

}
}
}
