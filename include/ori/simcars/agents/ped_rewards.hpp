#pragma once

#include <ori/simcars/geometry/defines.hpp>

#include <iostream>

namespace ori
{
namespace simcars
{
namespace agents
{

struct PedRewards
{
    FP_DATA_TYPE task_goal_reward;
    FP_DATA_TYPE space_reward;

    friend std::ostream& operator<<(std::ostream& output_stream, const PedRewards& rewards)
    {
        return output_stream << "Task Goal Reward = " << rewards.task_goal_reward << ", " <<
                                "Space Reward = " << rewards.space_reward;
    }
};

}
}
}
