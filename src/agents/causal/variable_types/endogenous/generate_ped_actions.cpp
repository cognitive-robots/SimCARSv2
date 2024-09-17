
#include <ori/simcars/structures/stl/stl_ordered_set.hpp>

#include <ori/simcars/agents/causal/variable_types/endogenous/generate_ped_actions.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

bool GeneratePedActionsVariable::get_value(structures::stl::STLStackArray<PedAction> &val) const
{
    structures::stl::STLStackArray<temporal::Time> time_range;
    structures::stl::STLStackArray<uint64_t> node_range;
    if (get_endogenous_parent()->get_value(time_range) &&
            get_other_parent()->get_value(node_range))
    {
        val.clear();

        for (size_t j = 0; j < node_range.count(); ++j)
        {
            Goal<uint64_t> node_goal;
            node_goal.val = node_range[j];

            for (size_t l = 0; l < time_range.count(); ++l)
            {
                node_goal.time = time_range[l];

                PedAction action;
                action.node_goal = node_goal;

                val.push_back(action);
            }
        }

        return true;
    }
    else
    {
        return false;
    }
}

bool GeneratePedActionsVariable::set_value(structures::stl::STLStackArray<PedAction> const &val)
{
    structures::stl::STLOrderedSet<temporal::Time> times;
    structures::stl::STLOrderedSet<uint64_t> nodes;

    for (size_t i = 0; i < val.count(); ++i)
    {
        PedAction action = val[i];
        Goal<uint64_t> node_goal = action.node_goal;
        uint64_t node_goal_val = node_goal.val;
        temporal::Time node_goal_time = node_goal.time;

        times.insert(node_goal_time);
        nodes.insert(node_goal_val);
    }

    return get_endogenous_parent()->set_value(times.get_array()) &&
            get_other_parent()->set_value(nodes.get_array());
}

}
}
}
}
