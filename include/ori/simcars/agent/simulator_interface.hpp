#pragma once

#include <ori/simcars/temporal/typedefs.hpp>
#include <ori/simcars/agent/state_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class ISimulator
{
public:
    virtual ~ISimulator() = default;

    virtual void simulate(agent::IState const *current_state, agent::IState *next_state, temporal::Duration time_step) const = 0;
};

}
}
}
