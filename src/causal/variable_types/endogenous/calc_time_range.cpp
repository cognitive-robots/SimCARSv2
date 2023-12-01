
#include <ori/simcars/causal/variable_types/endogenous/calc_time_range.hpp>

#include <ori/simcars/causal/variable_context.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

structures::stl::STLStackArray<temporal::Time> CalcTimeRange::get_value() const
{
    temporal::Time current = VariableContext::get_current_time();

    FP_DATA_TYPE horizon_secs = get_endogenous_parent()->get_value();
    temporal::Duration horizon =
            std::chrono::duration_cast<temporal::Duration>(
                std::chrono::duration<FP_DATA_TYPE>(horizon_secs));

    temporal::Time end = current + horizon;

    FP_DATA_TYPE interval_secs = get_other_parent()->get_value();
    temporal::Duration interval =
            std::chrono::duration_cast<temporal::Duration>(
                std::chrono::duration<FP_DATA_TYPE>(interval_secs));

    structures::stl::STLStackArray<temporal::Time> time_range;

    for (; current <= end; current += interval)
    {
        time_range.push_back(current);
    }

    return time_range;
}

}
}
}
