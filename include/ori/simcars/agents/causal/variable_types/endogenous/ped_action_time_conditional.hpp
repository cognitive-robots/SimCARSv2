#pragma once

#include <ori/simcars/geometry/defines.hpp>
#include <ori/simcars/temporal/typedefs.hpp>
#include <ori/simcars/causal/binary_endogenous_variable_abstract.hpp>
#include <ori/simcars/agents/ped_action.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

class PedActionTimeConditionalVariable :
        public simcars::causal::ABinaryEndogenousVariable<PedAction, PedAction, PedAction>
{
    temporal::Time const time;

public:
    PedActionTimeConditionalVariable(IEndogenousVariable<PedAction> *endogenous_parent,
                                     IVariable<PedAction> *other_parent, temporal::Time time);

    bool get_value(PedAction &val) const override;

    bool set_value(PedAction const &val) override;
};

}
}
}
}
