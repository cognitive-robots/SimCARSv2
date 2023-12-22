
#include <ori/simcars/causal/variable_types/exogenous/time_fixed.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

TimeFixedVariable::TimeFixedVariable(temporal::Time value) : value(value) {}

bool TimeFixedVariable::get_value(temporal::Time &val) const
{
    val = value;
    return true;
}

bool TimeFixedVariable::set_value(temporal::Time const &val)
{
    value = val;
    return true;
}

}
}
}
