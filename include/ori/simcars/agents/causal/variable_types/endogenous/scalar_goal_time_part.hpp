#pragma once

#include <ori/simcars/geometry/defines.hpp>
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

class ScalarGoalTimePartVariable :
        public simcars::causal::AUnaryEndogenousVariable<temporal::Time, Goal<FP_DATA_TYPE>>
{
public:
    using simcars::causal::AUnaryEndogenousVariable<temporal::Time, Goal<FP_DATA_TYPE>>::AUnaryEndogenousVariable;

    temporal::Time get_value() const override;
};

}
}
}
}
