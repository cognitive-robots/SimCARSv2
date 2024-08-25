#pragma once

#include <ori/simcars/geometry/defines.hpp>
#include <ori/simcars/temporal/typedefs.hpp>
#include <ori/simcars/causal/binary_endogenous_variable_abstract.hpp>
#include <ori/simcars/agents/fwd_car_action.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

class FWDCarActionTimeConditionalVariable :
        public simcars::causal::ABinaryEndogenousVariable<FWDCarAction, FWDCarAction, FWDCarAction>
{
    temporal::Time const time;

public:
    FWDCarActionTimeConditionalVariable(IEndogenousVariable<FWDCarAction> *endogenous_parent,
                                        IVariable<FWDCarAction> *other_parent, temporal::Time time);

    bool get_value(FWDCarAction &val) const override;

    bool set_value(FWDCarAction const &val) override;
};

}
}
}
}
