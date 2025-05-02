#pragma once

#include <ori/simcars/agents/ped_action.hpp>
#include <ori/simcars/agents/ped_outcome.hpp>
#include <ori/simcars/agents/ped_outcome_calc_parameters.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class IPedOutcomeCalc
{
public:
    virtual PedOutcome calc_outcome(PedOutcomeCalcParameters const *parameters) const = 0;
    virtual PedAction get_action() const = 0;
};

}
}
}
