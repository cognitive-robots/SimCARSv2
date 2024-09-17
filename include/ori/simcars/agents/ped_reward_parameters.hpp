#pragma once

#include <ori/simcars/geometry/defines.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

struct PedRewardParameters
{
    FP_DATA_TYPE task_goal_weight;
    FP_DATA_TYPE space_weight;
    FP_DATA_TYPE bias_weight;
};

}
}
}
