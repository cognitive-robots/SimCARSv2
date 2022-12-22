#pragma once

#include <ori/simcars/agent/simulated_valueless_variable_interface.hpp>
#include <ori/simcars/agent/variable_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

template <typename T>
class ISimulatedVariable : public virtual IVariable<T>, public virtual ISimulatedValuelessVariable
{
public:
    virtual ISimulatedVariable<T>* simulated_variable_deep_copy() const = 0;
};

}
}
}
