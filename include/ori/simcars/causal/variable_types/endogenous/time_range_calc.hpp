#pragma once

#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/geometry/defines.hpp>
#include <ori/simcars/temporal/typedefs.hpp>
#include <ori/simcars/causal/binary_endogenous_variable_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

class TimeRangeCalc :
        public ABinaryEndogenousVariable<structures::stl::STLStackArray<temporal::Time>,
        FP_DATA_TYPE, FP_DATA_TYPE>
{
public:
    using ABinaryEndogenousVariable<structures::stl::STLStackArray<temporal::Time>, FP_DATA_TYPE,
    FP_DATA_TYPE>::ABinaryEndogenousVariable;

    bool get_value(structures::stl::STLStackArray<temporal::Time> &val) const override;

    bool set_value(structures::stl::STLStackArray<temporal::Time> const &val) override;
};

}
}
}
