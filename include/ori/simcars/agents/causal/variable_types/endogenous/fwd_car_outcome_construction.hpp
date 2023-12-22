#pragma once

#include <ori/simcars/causal/ternary_endogenous_variable_abstract.hpp>
#include <ori/simcars/agents/fwd_car_outcome.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

class FWDCarOutcomeConstructionVariable :
        public ori::simcars::causal::ATernaryEndogenousVariable<FWDCarOutcome, int8_t,
        FP_DATA_TYPE, FP_DATA_TYPE>
{
public:
    using ori::simcars::causal::ATernaryEndogenousVariable<FWDCarOutcome, int8_t, FP_DATA_TYPE,
    FP_DATA_TYPE>::ATernaryEndogenousVariable;

    bool get_value(FWDCarOutcome &val) const override;
};

}
}
}
}
