#pragma once

#include <ori/simcars/causal/unary_endogenous_variable_abstract.hpp>
#include <ori/simcars/agents/typedefs.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

class RewardPedOutcomeActionTupleRewardPartVariable :
        public simcars::causal::AUnaryEndogenousVariable<FP_DATA_TYPE, RewardPedOutcomeActionTuple>
{
public:
    using simcars::causal::AUnaryEndogenousVariable<FP_DATA_TYPE, RewardPedOutcomeActionTuple>::AUnaryEndogenousVariable;

    bool get_value(FP_DATA_TYPE &val) const override;

    bool set_value(FP_DATA_TYPE const &val) override;
};

}
}
}
}
