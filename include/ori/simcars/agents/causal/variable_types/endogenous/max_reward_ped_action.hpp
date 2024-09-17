#pragma once

#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/causal/unary_endogenous_variable_abstract.hpp>
#include <ori/simcars/agents/typedefs.hpp>
#include <ori/simcars/agents/ped_action.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

class MaxRewardPedActionVariable :
        public simcars::causal::AUnaryEndogenousVariable<PedOutcomeActionPair,
        RewardPedOutcomeActionTuples>
{
public:
    using simcars::causal::AUnaryEndogenousVariable<PedOutcomeActionPair,
    RewardPedOutcomeActionTuples>::AUnaryEndogenousVariable;

    bool get_value(PedOutcomeActionPair &val) const override;

    bool set_value(PedOutcomeActionPair const &val) override;
};

}
}
}
}
