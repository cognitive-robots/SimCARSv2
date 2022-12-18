#pragma once

#include <ori/simcars/temporal/typedefs.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

template <typename T>
class IActionSampler
{
public:
    virtual ~IActionSampler() = default;

    virtual void sample_action(temporal::Time time_window_start,
                               temporal::Time time_window_end,
                               T const &goal_value_min,
                               T const &goal_value_max,
                               T &sampled_goal_value,
                               temporal::Time &sampled_action_start_time,
                               temporal::Time &sampled_action_end_time) const = 0;
};

}
}
}
