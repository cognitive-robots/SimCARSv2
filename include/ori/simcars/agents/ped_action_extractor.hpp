#pragma once

#include <ori/simcars/structures/dictionary_interface.hpp>
#include <ori/simcars/map/ped_map_interface.hpp>
#include <ori/simcars/agents/typedefs.hpp>
#include <ori/simcars/agents/ped_action.hpp>
#include <ori/simcars/agents/ped.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class PedActionExtractor
{
    map::IPedMap *map;
    temporal::Duration resolution;
    temporal::Duration action_min_duration;
    temporal::Duration node_min_duration;

    structures::IArray<TimeGoalPair<uint64_t>>* extract_node_goals(agents::Ped *ped) const;

public:
    PedActionExtractor(map::IPedMap *map, temporal::Duration resolution,
                          temporal::Duration action_min_duration,
                          temporal::Duration node_min_duration);

    ~PedActionExtractor() = default;

    structures::IArray<TimePedActionPair>* extract_actions(agents::Ped *ped) const;
};

}
}
}
