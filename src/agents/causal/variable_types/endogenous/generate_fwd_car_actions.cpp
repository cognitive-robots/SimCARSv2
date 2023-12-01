
#include <ori/simcars/agents/causal/variable_types/endogenous/generate_fwd_car_actions.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

structures::stl::STLStackArray<FWDCarAction> GenerateFWDCarAction::get_value() const
{
    structures::stl::STLStackArray<temporal::Time> time_range =
            get_endogenous_parent_1()->get_value();
    structures::stl::STLStackArray<FP_DATA_TYPE> speed_range =
            get_endogenous_parent_2()->get_value();
    structures::stl::STLStackArray<uint64_t> lane_range =
            get_other_parent()->get_value();

    structures::stl::STLStackArray<FWDCarAction> actions;

    for (size_t i = 0; i < speed_range.count(); ++i)
    {
        Goal<FP_DATA_TYPE> speed_goal;
        speed_goal.val = speed_range[i];

        for (size_t j = 0; j < lane_range.count(); ++j)
        {
            Goal<uint64_t> lane_goal;
            lane_goal.val = lane_range[j];

            for (size_t k = 0; k < time_range.count(); ++k)
            {
                speed_goal.time = time_range[k];

                for (size_t l = 0; l < time_range.count(); ++l)
                {
                    lane_goal.time = time_range[l];

                    FWDCarAction action;
                    action.speed_goal = speed_goal;
                    action.lane_goal = lane_goal;

                    actions.push_back(action);
                }
            }

        }
    }

    return actions;
}

}
}
}
}
