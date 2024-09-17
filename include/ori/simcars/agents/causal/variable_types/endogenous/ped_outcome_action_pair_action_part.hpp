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

class PedOutcomeActionPairActionPartVariable :
        public simcars::causal::AUnaryEndogenousVariable<PedAction, PedOutcomeActionPair>
{
public:
    using simcars::causal::AUnaryEndogenousVariable<PedAction, PedOutcomeActionPair>::AUnaryEndogenousVariable;

    bool get_value(PedAction &val) const override;

    bool set_value(PedAction const &val) override;
};

}
}
}
}
