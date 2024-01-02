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

class FWDCarActionLanePartVariable :
        public simcars::causal::AUnaryEndogenousVariable<Goal<uint64_t>, FWDCarAction>
{
public:
    using simcars::causal::AUnaryEndogenousVariable<Goal<uint64_t>, FWDCarAction>::AUnaryEndogenousVariable;

    bool get_value(Goal<uint64_t> &val) const override;

    bool set_value(Goal<uint64_t> const &val) override;
};

}
}
}
}
