
#include <ori/simcars/causal/variable_types/endogenous/duration_seconds_cast.hpp>

namespace ori
{
namespace simcars
{
namespace causal
{

FP_DATA_TYPE DurationSecondsCastVariable::get_value() const
{
    return std::chrono::duration_cast<std::chrono::seconds>(get_parent()->get_value()).count();
}

}
}
}
