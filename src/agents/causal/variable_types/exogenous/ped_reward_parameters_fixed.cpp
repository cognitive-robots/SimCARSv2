
#include <ori/simcars/agents/causal/variable_types/exogenous/ped_reward_parameters_fixed.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

PedRewardParametersFixedVariable::PedRewardParametersFixedVariable(PedRewardParameters value) :
    value(value) {}

bool PedRewardParametersFixedVariable::get_value(PedRewardParameters &val) const
{
    val = value;
    return true;
}

bool PedRewardParametersFixedVariable::set_value(PedRewardParameters const &val)
{
    value = val;
    return true;
}

}
}
}
}
