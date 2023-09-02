#pragma once

#include <ori/simcars/geometry/defines.hpp>
#include <ori/simcars/causal/unary_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class ScalarTimeStepSizeQuotientVariable :
        public AUnaryEndogenousVariable<FP_DATA_TYPE, FP_DATA_TYPE>
{
public:
    using AUnaryEndogenousVariable<FP_DATA_TYPE, FP_DATA_TYPE>::AUnaryEndogenousVariable;

    FP_DATA_TYPE get_value() const override;
};

}
}
}
