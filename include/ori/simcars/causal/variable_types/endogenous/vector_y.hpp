#pragma once

#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/causal/unary_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class VectorYVariable : public AUnaryEndogenousVariable<FP_DATA_TYPE, geometry::Vec>
{
public:
    using AUnaryEndogenousVariable<FP_DATA_TYPE, geometry::Vec>::AUnaryEndogenousVariable;

    FP_DATA_TYPE get_value() const override;
};

}
}
}
