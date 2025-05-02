
#include <ori/simcars/agents/causal/variable_types/exogenous/fwd_car_sim_parameters_fixed.hpp>

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

bool FWDCarSimParametersFixedVariable::get_value(FWDCarSimParameters &val) const
{
    val = value;
    return true;
}

bool FWDCarSimParametersFixedVariable::set_value(FWDCarSimParameters const &val)
{
    value = val;
    return true;
}

}
}
}
}
