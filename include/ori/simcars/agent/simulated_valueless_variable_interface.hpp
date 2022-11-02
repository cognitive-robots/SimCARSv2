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
    virtual std::shared_ptr<ISimulatedValuelessVariable> simulated_valueless_deep_copy() const = 0;

    virtual bool simulation_update(temporal::Time time, std::shared_ptr<const IState> state) const = 0;
};

}
}
}
