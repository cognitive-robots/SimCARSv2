#pragma once

#include <ori/simcars/geometry/defines.hpp>
#include <ori/simcars/causal/binary_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class ScalarBinaryMeanVariable :
        public ABinaryEndogenousVariable<FP_DATA_TYPE, FP_DATA_TYPE, FP_DATA_TYPE>
{
public:
    using ABinaryEndogenousVariable<FP_DATA_TYPE, FP_DATA_TYPE, FP_DATA_TYPE>::ABinaryEndogenousVariable;

    bool get_value(FP_DATA_TYPE &val) const override;

    bool set_value(FP_DATA_TYPE const &val) override;
};

}
}
}
