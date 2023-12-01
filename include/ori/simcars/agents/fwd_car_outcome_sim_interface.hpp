#pragma once

#include <ori/simcars/agents/fwd_car_action.hpp>
#include <ori/simcars/agents/fwd_car_outcome.hpp>
#include <ori/simcars/agents/fwd_car_sim_parameters.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class IFWDCarOutcomeSim
{
public:
    virtual FWDCarOutcome sim_outcome(FWDCarAction const *action,
                                      FWDCarSimParameters const *parameters) const = 0;
};

}
}
}
