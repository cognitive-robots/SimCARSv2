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

class ScalarGoalValPartVariable :
        public simcars::causal::AUnaryEndogenousVariable<FP_DATA_TYPE, Goal<FP_DATA_TYPE>>
{
public:
    using simcars::causal::AUnaryEndogenousVariable<FP_DATA_TYPE, Goal<FP_DATA_TYPE>>::AUnaryEndogenousVariable;

    bool get_value(FP_DATA_TYPE &val) const override;

    bool set_value(FP_DATA_TYPE const &val) override;
};

}
}
}
}
