#pragma once

#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/causal/binary_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class VectorDotProductVariable :
        public ABinaryEndogenousVariable<FP_DATA_TYPE, geometry::Vec, geometry::Vec>
{
public:
    using ABinaryEndogenousVariable<FP_DATA_TYPE, geometry::Vec, geometry::Vec>::ABinaryEndogenousVariable;

    FP_DATA_TYPE get_value() const override;
};

}
}
}
