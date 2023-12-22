
#include <ori/simcars/causal/variable_types/endogenous/scalar_range_calc.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

bool ScalarRangeCalc::get_value(structures::stl::STLStackArray<FP_DATA_TYPE> &val) const
{
    FP_DATA_TYPE current;
    FP_DATA_TYPE end;
    FP_DATA_TYPE interval;

    if (get_endogenous_parent_1()->get_value(current) &&
            get_endogenous_parent_2()->get_value(end) &&
            get_other_parent()->get_value(interval))
    {
        val.clear();

        for (; current <= end; current += interval)
        {
            val.push_back(current);
        }

        return true;
    }
    else
    {
        return false;
    }
}

bool ScalarRangeCalc::set_value(structures::stl::STLStackArray<FP_DATA_TYPE> const &val)
{
    if (val.count() < 2)
    {
        return false;
    }

    FP_DATA_TYPE current = val[0];
    FP_DATA_TYPE end = val[val.count() - 1];
    FP_DATA_TYPE interval = val[1] - val[0];

    for (size_t i = 0; i < val.count() - 1; ++i)
    {
        if (val[i + 1] - val[i] != interval)
        {
            return false;
        }
    }

    return get_endogenous_parent_1()->set_value(current) &&
            get_endogenous_parent_2()->set_value(end) &&
            get_other_parent()->set_value(interval);
}

}
}
}
