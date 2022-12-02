#pragma once

#include <ori/simcars/structures/stl/stl_dictionary.hpp>
#include <ori/simcars/agent/driving_scene_interface.hpp>
#include <ori/simcars/agent/read_only_driving_scene_state_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class BasicReadOnlyDrivingSceneState : public virtual AReadOnlyDrivingSceneState
{
    bool delete_dicts;

    structures::stl::STLDictionary<std::string, IReadOnlyDrivingAgentState const*> driving_agent_state_dict;

public:
    BasicReadOnlyDrivingSceneState(IDrivingScene const *scene, temporal::Time time, bool delete_dicts = true);
    BasicReadOnlyDrivingSceneState(IReadOnlyDrivingSceneState *driving_scene_state, bool copy_parameters = true);

    ~BasicReadOnlyDrivingSceneState();

    structures::IArray<IReadOnlyDrivingAgentState const*>* get_driving_agent_states() const override;
    IReadOnlyDrivingAgentState const* get_driving_agent_state(std::string const &driving_agent_name) const override;
};

}
}
}
