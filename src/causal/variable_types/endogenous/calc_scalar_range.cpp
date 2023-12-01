
#include <ori/simcars/causal/variable_types/endogenous/calc_scalar_range.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

structures::stl::STLStackArray<FP_DATA_TYPE> CalcScalarRange::get_value() const
{
    FP_DATA_TYPE current = get_endogenous_parent_1()->get_value();
    FP_DATA_TYPE end = get_endogenous_parent_2()->get_value();
    FP_DATA_TYPE interval = get_other_parent()->get_value();

    structures::stl::STLStackArray<FP_DATA_TYPE> scalar_range;

    for (; current <= end; current += interval)
    {
        scalar_range.push_back(current);
    }

    return scalar_range;
}

}
}
}
