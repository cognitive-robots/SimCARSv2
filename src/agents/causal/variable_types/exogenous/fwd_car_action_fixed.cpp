
#include <ori/simcars/agents/causal/variable_types/exogenous/fwd_car_action_fixed.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

FWDCarActionFixedVariable::FWDCarActionFixedVariable(FWDCarAction value) : value(value) {}

bool FWDCarActionFixedVariable::get_value(FWDCarAction &val) const
{
    val = value;
    return true;
}

bool FWDCarActionFixedVariable::set_value(FWDCarAction const &val)
{
    value = val;
    return true;
}

}
}
}
}
