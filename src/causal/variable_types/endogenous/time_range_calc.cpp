
#include <ori/simcars/causal/variable_types/endogenous/time_range_calc.hpp>

#include <ori/simcars/causal/variable_context.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

bool TimeRangeCalc::get_value(structures::stl::STLStackArray<temporal::Time> &val) const
{
    FP_DATA_TYPE horizon_secs;
    FP_DATA_TYPE interval_secs;

    if (get_endogenous_parent()->get_value(horizon_secs) &&
            get_other_parent()->get_value(interval_secs))
    {
        temporal::Time current = VariableContext::get_current_time();
        temporal::Time end = current + std::chrono::duration_cast<temporal::Duration>(
                    std::chrono::duration<FP_DATA_TYPE>(horizon_secs));
        temporal::Duration interval = std::chrono::duration_cast<temporal::Duration>(
                    std::chrono::duration<FP_DATA_TYPE>(interval_secs));

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

bool TimeRangeCalc::set_value(structures::stl::STLStackArray<temporal::Time> const &val)
{
    if (val.count() < 2)
    {
        return false;
    }

    temporal::Time start = val[0];
    temporal::Time end = val[val.count() - 1];
    temporal::Duration interval = val[1] - val[0];

    if (start != VariableContext::get_current_time())
    {
        return false;
    }

    for (size_t i = 0; i < val.count() - 1; ++i)
    {
        if (val[i + 1] - val[i] != interval)
        {
            return false;
        }
    }

    FP_DATA_TYPE horizon_secs =
            std::chrono::duration_cast<std::chrono::duration<FP_DATA_TYPE>>(end - start).count();
    FP_DATA_TYPE interval_secs =
            std::chrono::duration_cast<std::chrono::duration<FP_DATA_TYPE>>(interval).count();

    return get_endogenous_parent()->set_value(horizon_secs) &&
            get_other_parent()->set_value(interval_secs);
}

}
}
}
