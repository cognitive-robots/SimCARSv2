#pragma once

#include <ori/simcars/geometry/defines.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

struct PedSimParameters
{
    FP_DATA_TYPE sim_horizon_secs;
    FP_DATA_TYPE action_done_node_dist_threshold;
};

}
}
}
