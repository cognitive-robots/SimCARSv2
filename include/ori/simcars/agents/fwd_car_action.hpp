#pragma once

#include <ori/simcars/geometry/defines.hpp>
#include <ori/simcars/agents/goal.hpp>

#include <cmath>
#include <iostream>

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

    FWDCarAction(Goal<FP_DATA_TYPE> speed_goal = Goal<FP_DATA_TYPE>(),
                 Goal<uint64_t> lane_goal = Goal<uint64_t>()) : speed_goal(speed_goal),
        lane_goal(lane_goal) {}

    friend bool operator ==(FWDCarAction const &action_1, FWDCarAction const &action_2)
    {
        return action_1.speed_goal == action_2.speed_goal && action_1.lane_goal == action_2.lane_goal;
    }

    friend std::ostream& operator<<(std::ostream& output_stream, const FWDCarAction& action)
    {
        return output_stream << "Speed = " << std::to_string(action.speed_goal.val) << " m/s @ " <<
              std::to_string(action.speed_goal.time.time_since_epoch().count() / 1000) << " s, " <<
              "Lane = " << std::to_string(action.lane_goal.val) << " @ " <<
              std::to_string(action.lane_goal.time.time_since_epoch().count() / 1000) << " s";
    }

    friend FP_DATA_TYPE diff(FWDCarAction const &action_1, FWDCarAction const &action_2,
                             FP_DATA_TYPE scale = 0.1, FP_DATA_TYPE lane_diff_penalty = 10.0)
    {
        FP_DATA_TYPE speed_goal_val_diff = std::pow((action_1.speed_goal.val - action_2.speed_goal.val) /
                                                    (0.5 * (action_1.speed_goal.val + action_2.speed_goal.val)), 2);
        FP_DATA_TYPE speed_goal_time_diff = std::pow(std::chrono::duration_cast<std::chrono::duration<FP_DATA_TYPE>>(
                                                         action_1.speed_goal.time - action_2.speed_goal.time).count(), 2);
        FP_DATA_TYPE lane_goal_val_diff = action_1.lane_goal.val != action_2.lane_goal.val ? lane_diff_penalty : 0;
        FP_DATA_TYPE lane_goal_time_diff = std::pow(std::chrono::duration_cast<std::chrono::duration<FP_DATA_TYPE>>(
                                                        action_1.lane_goal.time - action_2.lane_goal.time).count(), 2);
        return scale * std::sqrt(speed_goal_val_diff + speed_goal_time_diff + lane_goal_val_diff +
                                 lane_goal_time_diff);
    }
};

}
}
}
