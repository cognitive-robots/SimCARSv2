#pragma once

#include <ori/simcars/causal/unary_endogenous_variable_abstract.hpp>
#include <ori/simcars/agents/goal.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

class IdGoalValPartVariable :
        public simcars::causal::AUnaryEndogenousVariable<uint64_t, Goal<uint64_t>>
{
public:
    using simcars::causal::AUnaryEndogenousVariable<uint64_t, Goal<uint64_t>>::AUnaryEndogenousVariable;

    uint64_t get_value() const override;
};

}
}
}
}
