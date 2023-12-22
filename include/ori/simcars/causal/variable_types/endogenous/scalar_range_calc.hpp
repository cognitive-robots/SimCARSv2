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

class ScalarRangeCalc :
        public ATernaryEndogenousVariable<structures::stl::STLStackArray<FP_DATA_TYPE>, FP_DATA_TYPE, FP_DATA_TYPE, FP_DATA_TYPE>
{
public:
    using ATernaryEndogenousVariable<structures::stl::STLStackArray<FP_DATA_TYPE>, FP_DATA_TYPE, FP_DATA_TYPE, FP_DATA_TYPE>::ATernaryEndogenousVariable;

    bool get_value(structures::stl::STLStackArray<FP_DATA_TYPE> &val) const override;

    bool set_value(structures::stl::STLStackArray<FP_DATA_TYPE> const &val) override;
};

}
}
}
