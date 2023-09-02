#pragma once

#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/geometry/trig_buff.hpp>
#include <ori/simcars/causal/binary_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class VectorXYConstructionVariable : public ABinaryEndogenousVariable<geometry::Vec, FP_DATA_TYPE, FP_DATA_TYPE>
{
public:
    using ABinaryEndogenousVariable<geometry::Vec, FP_DATA_TYPE, FP_DATA_TYPE>::ABinaryEndogenousVariable;

    geometry::Vec get_value() const override;
};

}
}
}
