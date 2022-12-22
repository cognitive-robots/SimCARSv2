#pragma once

#include <ori/simcars/agent/simulated_variable_interface.hpp>
#include <ori/simcars/agent/variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

template <typename T>
class ASimulatedVariable : public virtual AVariable<T>, public virtual ISimulatedVariable<T>
{
public:
    IVariable<T>* variable_deep_copy() const override
    {
        return this->simulated_variable_deep_copy();
    }

    ISimulatedValuelessVariable* simulated_valueless_variable_deep_copy() const override
    {
        return this->simulated_variable_deep_copy();
    }
};

}
}
}
