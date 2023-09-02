#pragma once

#include <ori/simcars/geometry/defines.hpp>
#include <ori/simcars/causal/binary_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class ScalarProductVariable :
        public ABinaryEndogenousVariable<FP_DATA_TYPE, FP_DATA_TYPE, FP_DATA_TYPE>
{
public:
    using ABinaryEndogenousVariable<FP_DATA_TYPE, FP_DATA_TYPE, FP_DATA_TYPE>::ABinaryEndogenousVariable;

    FP_DATA_TYPE get_value() const override;
};

}
}
}
