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

    bool get_value(uint64_t &val) const override;

    bool set_value(uint64_t const &val) override;
};

}
}
}
}
