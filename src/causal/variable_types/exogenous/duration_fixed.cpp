
#include <ori/simcars/causal/variable_types/exogenous/duration_fixed.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

DurationFixedVariable::DurationFixedVariable(temporal::Duration value) : value(value) {}

bool DurationFixedVariable::get_value(temporal::Duration &val) const
{
    val = value;
    return true;
}

bool DurationFixedVariable::set_value(temporal::Duration const &val)
{
    value = val;
    return true;
}

}
}
}
