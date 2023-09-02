
#include <ori/simcars/causal/variable_types/exogenous/duration_fixed.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

DurationFixedVariable::DurationFixedVariable(temporal::Duration value) : value(value) {}

temporal::Duration DurationFixedVariable::get_value() const
{
    return value;
}

}
}
}
