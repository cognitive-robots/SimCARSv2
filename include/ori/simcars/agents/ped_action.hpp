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

struct PedAction
{
    Goal<uint64_t> node_goal;

    PedAction(Goal<uint64_t> node_goal = Goal<uint64_t>()) : node_goal(node_goal) {}

    friend bool operator ==(PedAction const &action_1, PedAction const &action_2)
    {
        return action_1.node_goal == action_2.node_goal;
    }

    friend std::ostream& operator<<(std::ostream& output_stream, const PedAction& action)
    {
        return output_stream << "Node = " << std::to_string(action.node_goal.val) << " @ " <<
                                std::to_string(action.node_goal.time.time_since_epoch().count() /
                                               1000) << " s";
    }

    friend FP_DATA_TYPE diff(PedAction const &action_1, PedAction const &action_2,
                             FP_DATA_TYPE scale = 0.1, FP_DATA_TYPE node_diff_penalty = 10.0)
    {
        FP_DATA_TYPE node_goal_val_diff =
                action_1.node_goal.val != action_2.node_goal.val ? node_diff_penalty : 0;
        FP_DATA_TYPE node_goal_time_diff =
                std::pow(std::chrono::duration_cast<std::chrono::duration<FP_DATA_TYPE>>(
                             action_1.node_goal.time - action_2.node_goal.time).count(), 2);
        return scale * std::sqrt(node_goal_val_diff + node_goal_time_diff);
    }
};

}
}
}
