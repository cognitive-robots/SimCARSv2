
#include <ori/simcars/causal/variable_types/exogenous/scalar_socket.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

ScalarSocketVariable::ScalarSocketVariable(FP_DATA_TYPE default_value,
                                           IVariable<FP_DATA_TYPE> const *parent) :
    default_value(default_value), parent(parent) {}

FP_DATA_TYPE ScalarSocketVariable::get_value() const
{
    if (parent != nullptr)
    {
        return parent->get_value();
    }
    else
    {
        return default_value;
    }
}

IVariable<FP_DATA_TYPE> const* ScalarSocketVariable::get_parent() const
{
    return parent;
}

void ScalarSocketVariable::set_parent(const IVariable<FP_DATA_TYPE> *parent)
{
    this->parent = parent;
}

}
}
}
