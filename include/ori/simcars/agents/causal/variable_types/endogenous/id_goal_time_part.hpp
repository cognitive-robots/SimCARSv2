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

class IdGoalTimePartVariable :
        public simcars::causal::AUnaryEndogenousVariable<temporal::Time, Goal<uint64_t>>
{
public:
    using simcars::causal::AUnaryEndogenousVariable<temporal::Time, Goal<uint64_t>>::AUnaryEndogenousVariable;

    bool get_value(temporal::Time &val) const override;
};

}
}
}
}
