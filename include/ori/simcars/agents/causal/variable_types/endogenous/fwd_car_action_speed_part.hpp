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

    Goal<FP_DATA_TYPE> get_value() const override;
};

}
}
}
}
