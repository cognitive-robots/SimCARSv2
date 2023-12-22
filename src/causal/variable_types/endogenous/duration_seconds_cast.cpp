
#include <ori/simcars/causal/variable_types/endogenous/duration_seconds_cast.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

bool DurationSecondsCastVariable::get_value(FP_DATA_TYPE &val) const
{
    temporal::Duration duration;
    if(get_parent()->get_value(duration))
    {
        val = std::chrono::duration_cast<std::chrono::duration<FP_DATA_TYPE>>(duration).count();
        return true;
    }
    else
    {
        return false;
    }
}

bool DurationSecondsCastVariable::set_value(FP_DATA_TYPE const &val)
{
    temporal::Duration duration;
    duration = std::chrono::duration_cast<temporal::Duration>(
                std::chrono::duration<FP_DATA_TYPE>(val));
    return get_parent()->set_value(duration);
}

}
}
}
