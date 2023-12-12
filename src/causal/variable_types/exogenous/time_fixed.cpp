
#include <ori/simcars/causal/variable_types/exogenous/time_fixed.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

TimeFixedVariable::TimeFixedVariable(temporal::Time value) : value(value) {}

temporal::Time TimeFixedVariable::get_value() const
{
    return value;
}

}
}
}
