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

class FWDCarOutcomeActionPairActionPartVariable :
        public simcars::causal::AUnaryEndogenousVariable<FWDCarAction, FWDCarOutcomeActionPair>
{
public:
    using simcars::causal::AUnaryEndogenousVariable<FWDCarAction, FWDCarOutcomeActionPair>::AUnaryEndogenousVariable;

    bool get_value(FWDCarAction &val) const override;

    bool set_value(FWDCarAction const &val) override;
};

}
}
}
}
