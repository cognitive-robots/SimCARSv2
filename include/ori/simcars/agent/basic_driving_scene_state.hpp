#pragma once

#include <ori/simcars/structures/stl/stl_dictionary.hpp>
#include <ori/simcars/agent/driving_scene_interface.hpp>
#include <ori/simcars/agent/driving_scene_state_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class BasicDrivingSceneState : public virtual ADrivingSceneState
{
    bool delete_dicts;

    structures::stl::STLDictionary<std::string, IDrivingAgentState*> driving_agent_state_dict;

public:
    BasicDrivingSceneState(bool delete_dicts = true);
    BasicDrivingSceneState(IDrivingScene *scene, temporal::Time time, bool delete_dicts = true);
    BasicDrivingSceneState(IDrivingSceneState *driving_scene_state, bool copy_parameters = true);

    ~BasicDrivingSceneState();

    structures::IArray<IReadOnlyDrivingAgentState const*>* get_driving_agent_states() const override;
    IReadOnlyDrivingAgentState const* get_driving_agent_state(std::string const &driving_agent_name) const override;

    structures::IArray<IDrivingAgentState*>* get_mutable_driving_agent_states() override;
    IDrivingAgentState* get_mutable_driving_agent_state(std::string const &driving_agent_name) override;
};

}
}
}
