#pragma once

#include <ori/simcars/geometry/defines.hpp>
#include <ori/simcars/agent/action_sampler_interface.hpp>

#include <random>

namespace ori
{
namespace simcars
{
namespace agent
{

class BasicFPActionSampler : public virtual IActionSampler<FP_DATA_TYPE>
{
    std::random_device random_device;
    mutable std::mt19937 randomness_generator;

public:
    BasicFPActionSampler();

    void sample_action(temporal::Time time_window_start,
                       temporal::Time time_window_end,
                       FP_DATA_TYPE const &goal_value_min,
                       FP_DATA_TYPE const &goal_value_max,
                       FP_DATA_TYPE &sampled_goal_value,
                       temporal::Time &sampled_action_start_time,
                       temporal::Time &sampled_action_end_time) const override;
};

}
}
}
