#pragma once

#include <ori/simcars/agent/valueless_variable_interface.hpp>
#include <ori/simcars/agent/state_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class ISimulatedValuelessVariable : public virtual IValuelessVariable
{
public:
    virtual ISimulatedValuelessVariable* simulated_valueless_deep_copy() const = 0;

    virtual bool simulation_update(temporal::Time time, IState const *state) const = 0;

    virtual void begin_simulation(temporal::Time simulation_start_time) const = 0;
};

}
}
}
