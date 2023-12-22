
#include <ori/simcars/causal/variable_types/exogenous/scalar_socket.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

ScalarSocketVariable::ScalarSocketVariable(FP_DATA_TYPE default_value,
                                           IVariable<FP_DATA_TYPE> *parent) :
    default_value(default_value), parent(parent) {}

bool ScalarSocketVariable::get_value(FP_DATA_TYPE &val) const
{
    if (parent != nullptr)
    {
        return parent->get_value(val);
    }
    else
    {
        val = default_value;
        return true;
    }
}

IVariable<FP_DATA_TYPE> const* ScalarSocketVariable::get_parent() const
{
    return parent;
}

bool ScalarSocketVariable::set_value(FP_DATA_TYPE const &val)
{
    if (parent != nullptr)
    {
        return parent->set_value(val);
    }
    else
    {
        default_value = val;
        return true;
    }
}

void ScalarSocketVariable::set_parent(IVariable<FP_DATA_TYPE> *parent)
{
    this->parent = parent;
}

}
}
}
