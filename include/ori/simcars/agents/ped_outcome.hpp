#pragma once

#include <ori/simcars/geometry/defines.hpp>

#include <cstdint>
#include <cmath>
#include <iostream>

namespace ori
{
namespace simcars
{
namespace agents
{

struct PedOutcome
{
    uint64_t node;
    bool action_done;

    friend bool operator ==(PedOutcome const &outcome_1, PedOutcome const &outcome_2)
    {
        return outcome_1.node == outcome_2.node &&
                outcome_1.action_done == outcome_2.action_done;
    }

    friend FP_DATA_TYPE diff(PedOutcome const &outcome_1, PedOutcome const &outcome_2,
                             FP_DATA_TYPE scale = 0.1,
                             FP_DATA_TYPE node_scale = 100.0,
                             FP_DATA_TYPE action_done_scale = 100.0)
    {

        FP_DATA_TYPE node_diff = std::pow(outcome_1.node - outcome_2.node, 2.0);
        FP_DATA_TYPE action_done_diff = std::pow(outcome_1.action_done -
                                                 outcome_2.action_done, 2.0);
        return scale * std::sqrt(node_scale * node_diff + action_done_scale * action_done_diff);
    }

    friend std::ostream& operator<<(std::ostream& output_stream, const PedOutcome& outcome)
    {
        return output_stream << "Node = " << outcome.node << ", " <<
                                "Action Done = " << outcome.action_done;
    }
};

}
}
}
