
#include <ori/simcars/agents/causal/variable_types/exogenous/fwd_car_outcome_parameters_fixed.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

FWDCarSimParametersFixedVariable::FWDCarSimParametersFixedVariable(FWDCarSimParameters value) :
    value(value) {}

FWDCarSimParameters FWDCarSimParametersFixedVariable::get_value() const
{
    return value;
}

}
}
}
}
