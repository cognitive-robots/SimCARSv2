
#include <ori/simcars/causal/variable_types/exogenous/time_socket.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

TimeSocketVariable::TimeSocketVariable(temporal::Time default_value,
                                       IVariable<temporal::Time> const *parent) :
    default_value(default_value), parent(parent) {}

temporal::Time TimeSocketVariable::get_value() const
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

IVariable<temporal::Time> const* TimeSocketVariable::get_parent() const
{
    return parent;
}

void TimeSocketVariable::set_parent(const IVariable<temporal::Time> *parent)
{
    this->parent = parent;
}

}
}
}
