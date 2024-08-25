#pragma once

#include <ori/simcars/geometry/typedefs.hpp>

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
    geometry::Vec pos;
    bool action_done;

    friend bool operator ==(PedOutcome const &outcome_1, PedOutcome const &outcome_2)
    {
        return outcome_1.pos == outcome_2.pos && outcome_1.action_done == outcome_2.action_done;
    }

    friend FP_DATA_TYPE diff(PedOutcome const &outcome_1, PedOutcome const &outcome_2,
                             FP_DATA_TYPE scale = 0.1,
                             FP_DATA_TYPE pos_scale = 1.0,
                             FP_DATA_TYPE action_done_scale = 100.0)
    {

        FP_DATA_TYPE pos_diff = (outcome_1.pos - outcome_2.pos).squaredNorm();
        FP_DATA_TYPE action_done_diff = std::pow(outcome_1.action_done -
                                                 outcome_2.action_done, 2.0);
        return scale * std::sqrt(pos_scale * pos_diff + action_done_scale * action_done_diff);
    }

    friend std::ostream& operator<<(std::ostream& output_stream, const PedOutcome& outcome)
    {
        return output_stream << "Position = " << outcome.pos << ", " <<
                                "Action Done = " << outcome.action_done;
    }
};

}
}
}
