#pragma once

#include <ori/simcars/causal/unary_endogenous_variable_abstract.hpp>
#include <ori/simcars/agents/fwd_car_action.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

class FWDCarActionSpeedPartVariable :
        public simcars::causal::AUnaryEndogenousVariable<Goal<FP_DATA_TYPE>, FWDCarAction>
{
public:
    using simcars::causal::AUnaryEndogenousVariable<Goal<FP_DATA_TYPE>, FWDCarAction>::AUnaryEndogenousVariable;

    bool get_value(Goal<FP_DATA_TYPE> &val) const override;

    bool set_value(Goal<FP_DATA_TYPE> const &val) override;
};

}
}
}
}
