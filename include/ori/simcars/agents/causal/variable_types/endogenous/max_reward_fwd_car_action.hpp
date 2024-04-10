#pragma once

#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/causal/unary_endogenous_variable_abstract.hpp>
#include <ori/simcars/agents/typedefs.hpp>
#include <ori/simcars/agents/fwd_car_action.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

class MaxRewardFWDCarActionVariable :
        public simcars::causal::AUnaryEndogenousVariable<FWDCarOutcomeActionPair,
        RewardFWDCarOutcomeActionTuples>
{
public:
    using simcars::causal::AUnaryEndogenousVariable<FWDCarOutcomeActionPair,
    RewardFWDCarOutcomeActionTuples>::AUnaryEndogenousVariable;

    bool get_value(FWDCarOutcomeActionPair &val) const override;

    bool set_value(FWDCarOutcomeActionPair const &val) override;
};

}
}
}
}
