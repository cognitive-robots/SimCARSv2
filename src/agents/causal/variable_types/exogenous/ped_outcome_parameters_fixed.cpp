
#include <ori/simcars/agents/causal/variable_types/exogenous/ped_outcome_parameters_fixed.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

PedSimParametersFixedVariable::PedSimParametersFixedVariable(PedSimParameters value) :
    value(value) {}

bool PedSimParametersFixedVariable::get_value(PedSimParameters &val) const
{
    val = value;
    return true;
}

bool PedSimParametersFixedVariable::set_value(PedSimParameters const &val)
{
    value = val;
    return true;
}

}
}
}
}
