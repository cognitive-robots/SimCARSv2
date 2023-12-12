#pragma once

#include <ori/simcars/geometry/defines.hpp>
#include <ori/simcars/temporal/typedefs.hpp>
#include <ori/simcars/causal/ternary_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class ScalarTimeConditionalVariable :
        public ATernaryEndogenousVariable<FP_DATA_TYPE, FP_DATA_TYPE, FP_DATA_TYPE, temporal::Time>
{
public:
    using ATernaryEndogenousVariable<FP_DATA_TYPE, FP_DATA_TYPE, FP_DATA_TYPE,
    temporal::Time>::ATernaryEndogenousVariable;

    FP_DATA_TYPE get_value() const override;
};

}
}
}
