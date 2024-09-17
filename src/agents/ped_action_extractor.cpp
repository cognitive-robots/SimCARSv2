
#include <ori/simcars/agents/ped_action_extractor.hpp>

#include <ori/simcars/structures/stl/stl_deque_array.hpp>
#include <ori/simcars/map/node_interface.hpp>
#include <ori/simcars/causal/variable_context.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

structures::IArray<TimeGoalPair<uint64_t>>* PedActionExtractor::extract_node_goals(
        agents::Ped *ped) const
{
    structures::stl::STLDequeArray<TimeGoalPair<uint64_t>> *node_goals =
            new structures::stl::STLDequeArray<TimeGoalPair<uint64_t>>;

    temporal::Time start_time = ped->get_min_time();
    temporal::Time end_time = ped->get_max_time();

    simcars::causal::IEndogenousVariable<geometry::Vec> *pos =
            ped->get_pos_variable();

    temporal::Time node_change_time = start_time +
            2 * simcars::causal::VariableContext::get_time_step_size();
    map::INode const *prev_node = nullptr;
    for (temporal::Time current_time = start_time;
         current_time <= end_time;
         current_time += resolution)
    {
        simcars::causal::VariableContext::set_current_time(current_time);

        geometry::Vec curr_pos;
        if (!pos->get_value(curr_pos))
        {
            continue;
        }

        map::INode const *curr_node = map->get_node(curr_pos);

        if (curr_node == nullptr)
        {
            continue;
        }

        if (prev_node != nullptr)
        {
            if (curr_node != prev_node)
            {
                    node_change_time = current_time;
                    prev_node = curr_node;
                    curr_node = nullptr;
            }
            else
            {
                prev_node = curr_node;
                curr_node = nullptr;
            }
        }
        else
        {
            // There were no previous node from which to calculate continuing node, so use
            // current nodes for updating the previous nodes.
            prev_node = curr_node;
            curr_node = nullptr;
        }

        if (node_change_time != temporal::Time::max())
        {
            if (node_goals->count() == 0)
            {
                // WARNING: Takes first node as the node goal
                Goal<uint64_t> node_goal(prev_node->get_id(), start_time +
                                         2 * simcars::causal::VariableContext::get_time_step_size());
                node_goals->push_back(
                            TimeGoalPair<uint64_t>(start_time +
                                                   2 * simcars::causal::VariableContext::get_time_step_size(),
                                                   node_goal));

                node_change_time = temporal::Time::max();
            }
            else if (current_time - node_change_time >= node_min_duration &&
                     current_time - (action_min_duration + node_min_duration) >
                     (*node_goals)[node_goals->count() - 1].second.time)
            {
                if ((*node_goals)[node_goals->count() - 1].second.val != prev_node->get_id())
                {
                    // WARNING: Takes first node as the node goal
                    Goal<uint64_t> node_goal(prev_node->get_id(), current_time - node_min_duration);
                    node_goals->push_back(TimeGoalPair<uint64_t>(current_time -
                                                                 (node_min_duration +
                                                                  action_min_duration),
                                                                 node_goal));
                }
                else
                {
                    // We ended up in the same node as we started with too little time in any other
                    // node to consider a node change to have occured. In other words, do nothing.
                }

                node_change_time = temporal::Time::max();
            }
            else
            {
                // We haven't been in this node long enough to consider it an actual node change.
                // Keep waiting!
            }
        }
    }

    if (node_goals->count() == 0)
    {
        throw std::runtime_error("Node is not available at start time of agent");
    }

    return node_goals;
}

PedActionExtractor::PedActionExtractor(map::IPedMap *map, temporal::Duration resolution,
                                       temporal::Duration action_min_duration,
                                       temporal::Duration node_min_duration) :
    map(map), resolution(resolution), action_min_duration(action_min_duration),
    node_min_duration(node_min_duration)
{
    assert(resolution >= simcars::causal::VariableContext::get_time_step_size());
}

structures::IArray<TimePedActionPair>* PedActionExtractor::extract_actions(agents::Ped *ped) const
{
    structures::stl::STLStackArray<TimePedActionPair> *actions =
            new structures::stl::STLStackArray<TimePedActionPair>;

    structures::IArray<TimeGoalPair<uint64_t>> *node_goals = extract_node_goals(ped);

    for (size_t i = 0; i < node_goals->count(); ++i)
    {
        PedAction action((*node_goals)[i].second);
        temporal::Time start_time = (*node_goals)[i].first;
        actions->push_back(TimePedActionPair(start_time, action));
    }

    delete node_goals;

    return actions;
}

}
}
}
