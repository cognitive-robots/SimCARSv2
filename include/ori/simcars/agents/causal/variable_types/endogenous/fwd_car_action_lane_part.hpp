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

class FWDCarActionLanePart :
        public simcars::causal::AUnaryEndogenousVariable<Goal<uint64_t>, FWDCarAction>
{
public:
    using simcars::causal::AUnaryEndogenousVariable<Goal<uint64_t>, FWDCarAction>::AUnaryEndogenousVariable;

    Goal<uint64_t> get_value() const override;
};

}
}
}
}
