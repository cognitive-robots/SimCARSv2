#pragma once

#include <ori/simcars/geometry/defines.hpp>
#include <ori/simcars/agents/goal.hpp>

#include <cmath>

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

    friend bool operator ==(FWDCarAction const &action_1, FWDCarAction const &action_2)
    {
        return action_1.speed_goal == action_2.speed_goal && action_1.lane_goal == action_2.lane_goal;
    }

    friend FP_DATA_TYPE diff(FWDCarAction const &action_1, FWDCarAction const &action_2,
                             FP_DATA_TYPE scale = 1.0, FP_DATA_TYPE lane_diff_penalty = 1.0)
    {
        return scale * std::sqrt(std::pow(action_1.speed_goal.val - action_2.speed_goal.val, 2) +
                                 std::pow(std::chrono::duration_cast<std::chrono::duration<FP_DATA_TYPE>>(
                                              action_1.speed_goal.time - action_2.speed_goal.time).count(), 2) +
                                 action_1.lane_goal.val == action_2.lane_goal.val ? lane_diff_penalty : 0 +
                                 std::pow(std::chrono::duration_cast<std::chrono::duration<FP_DATA_TYPE>>(
                                              action_1.lane_goal.time - action_2.lane_goal.time).count(), 2)
                         );
    }
};

}
}
}
