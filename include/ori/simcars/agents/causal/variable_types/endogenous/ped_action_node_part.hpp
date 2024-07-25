#pragma once

#include <ori/simcars/causal/unary_endogenous_variable_abstract.hpp>
#include <ori/simcars/agents/ped_action.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

class PedActionNodePartVariable :
        public simcars::causal::AUnaryEndogenousVariable<Goal<uint64_t>, PedAction>
{
public:
    using simcars::causal::AUnaryEndogenousVariable<Goal<uint64_t>, PedAction>::AUnaryEndogenousVariable;

    bool get_value(Goal<uint64_t> &val) const override;

    bool set_value(Goal<uint64_t> const &val) override;
};

}
}
}
}
