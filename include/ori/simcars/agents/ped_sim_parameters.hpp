#pragma once

#include <ori/simcars/geometry/defines.hpp>
#include <ori/simcars/agents/ped_outcome_calc_parameters.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

struct PedSimParameters : public PedOutcomeCalcParameters
{
    FP_DATA_TYPE sim_horizon_secs;
};

}
}
}
