
#include <ori/simcars/structures/stl/stl_ordered_set.hpp>

#include <ori/simcars/agents/causal/variable_types/endogenous/generate_fwd_car_actions.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

bool GenerateFWDCarActionsVariable::get_value(structures::stl::STLStackArray<FWDCarAction> &val) const
{
    structures::stl::STLStackArray<temporal::Time> time_range;
    structures::stl::STLStackArray<FP_DATA_TYPE> speed_range;
    structures::stl::STLStackArray<uint64_t> lane_range;
    if (get_endogenous_parent_1()->get_value(time_range) &&
            get_endogenous_parent_2()->get_value(speed_range) &&
            get_other_parent()->get_value(lane_range))
    {
        val.clear();

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

                        val.push_back(action);
                    }
                }

            }
        }

        return true;
    }
    else
    {
        return false;
    }
}

bool GenerateFWDCarActionsVariable::set_value(structures::stl::STLStackArray<FWDCarAction> const &val)
{
    structures::stl::STLOrderedSet<temporal::Time> times;
    structures::stl::STLOrderedSet<FP_DATA_TYPE> speeds;
    structures::stl::STLOrderedSet<uint64_t> lanes;

    for (size_t i = 0; i < val.count(); ++i)
    {
        FWDCarAction action = val[i];
        Goal<FP_DATA_TYPE> speed_goal = action.speed_goal;
        FP_DATA_TYPE speed_goal_val = speed_goal.val;
        temporal::Time speed_goal_time = speed_goal.time;
        Goal<uint64_t> lane_goal = action.lane_goal;
        uint64_t lane_goal_val = lane_goal.val;
        temporal::Time lane_goal_time = lane_goal.time;

        times.insert(speed_goal_time);
        times.insert(lane_goal_time);
        speeds.insert(speed_goal_val);
        lanes.insert(lane_goal_val);
    }

    return get_endogenous_parent_1()->set_value(times.get_array()) &&
            get_endogenous_parent_2()->set_value(speeds.get_array()) &&
            get_other_parent()->set_value(lanes.get_array());
}

}
}
}
}
