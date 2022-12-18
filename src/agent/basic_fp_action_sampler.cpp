
#include <ori/simcars/agent/basic_fp_action_sampler.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

BasicFPActionSampler::BasicFPActionSampler()
    : randomness_generator(random_device()) {}

void BasicFPActionSampler::sample_action(temporal::Time time_window_start,
                                         temporal::Time time_window_end,
                                         FP_DATA_TYPE const &goal_value_min,
                                         FP_DATA_TYPE const &goal_value_max,
                                         FP_DATA_TYPE &sampled_goal_value,
                                         temporal::Time &sampled_action_start_time,
                                         temporal::Time &sampled_action_end_time) const
{
    std::uniform_real_distribution<float> goal_value_generator(goal_value_min,
                                                               goal_value_max);
    std::uniform_int_distribution<int64_t> time_generator(
                time_window_start.time_since_epoch().count(),
                time_window_end.time_since_epoch().count());

    sampled_goal_value = goal_value_generator(randomness_generator);

    int64_t sampled_raw_time_1 = time_generator(randomness_generator);
    int64_t sampled_raw_time_2 = time_generator(randomness_generator);

    if (sampled_raw_time_1 <= sampled_raw_time_2)
    {
        sampled_action_start_time =
                temporal::Time(temporal::Duration(sampled_raw_time_1));
        sampled_action_end_time =
                temporal::Time(temporal::Duration(sampled_raw_time_2));
    }
    else
    {
        sampled_action_start_time =
                temporal::Time(temporal::Duration(sampled_raw_time_2));
        sampled_action_end_time =
                temporal::Time(temporal::Duration(sampled_raw_time_1));
    }
}

}
}
}
