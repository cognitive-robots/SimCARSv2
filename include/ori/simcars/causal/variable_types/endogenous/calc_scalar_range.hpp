#pragma once

#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/geometry/defines.hpp>
#include <ori/simcars/causal/ternary_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class CalcScalarRange :
        public ATernaryEndogenousVariable<structures::stl::STLStackArray<FP_DATA_TYPE>, FP_DATA_TYPE, FP_DATA_TYPE, FP_DATA_TYPE>
{
public:
    using ATernaryEndogenousVariable<structures::stl::STLStackArray<FP_DATA_TYPE>, FP_DATA_TYPE, FP_DATA_TYPE, FP_DATA_TYPE>::ATernaryEndogenousVariable;

    structures::stl::STLStackArray<FP_DATA_TYPE> get_value() const override;
};

}
}
}
