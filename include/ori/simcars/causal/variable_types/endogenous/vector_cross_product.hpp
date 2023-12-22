#pragma once

#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/causal/binary_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class VectorCrossProductVariable :
        public ABinaryEndogenousVariable<FP_DATA_TYPE, geometry::Vec, geometry::Vec>
{
public:
    using ABinaryEndogenousVariable<FP_DATA_TYPE, geometry::Vec, geometry::Vec>::ABinaryEndogenousVariable;

    bool get_value(FP_DATA_TYPE &val) const override;

    bool set_value(FP_DATA_TYPE const &val) override;
};

}
}
}
