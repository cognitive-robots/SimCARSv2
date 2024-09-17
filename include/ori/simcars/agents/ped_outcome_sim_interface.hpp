#pragma once

#include <ori/simcars/agents/ped_action.hpp>
#include <ori/simcars/agents/ped_outcome.hpp>
#include <ori/simcars/agents/ped_sim_parameters.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class IPedOutcomeSim
{
public:
    virtual PedOutcome sim_outcome(PedAction const *action,
                                   PedSimParameters const *parameters) const = 0;
};

}
}
}
