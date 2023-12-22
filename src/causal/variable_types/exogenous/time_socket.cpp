
#include <ori/simcars/causal/variable_types/exogenous/time_socket.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

TimeSocketVariable::TimeSocketVariable(temporal::Time default_value,
                                       IVariable<temporal::Time> *parent) :
    default_value(default_value), parent(parent) {}

bool TimeSocketVariable::get_value(temporal::Time &val) const
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

IVariable<temporal::Time> const* TimeSocketVariable::get_parent() const
{
    return parent;
}

bool TimeSocketVariable::set_value(temporal::Time const &val)
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

void TimeSocketVariable::set_parent(IVariable<temporal::Time> *parent)
{
    this->parent = parent;
}

}
}
}
