#pragma once

#include <ori/simcars/structures/stl/stl_dictionary.hpp>
#include <ori/simcars/agent/driving_agent_state_interface.hpp>
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

protected:
    structures::stl::STLDictionary<std::string, IDrivingAgentState const*> driving_agent_state_dict;

public:
    BasicDrivingSceneState(bool delete_dicts = true);
    BasicDrivingSceneState(IDrivingSceneState const *driving_scene_state, bool copy_parameters = true);

    ~BasicDrivingSceneState();

    structures::IArray<IDrivingAgentState const*>* get_driving_agent_states() const override;
    IDrivingAgentState const* get_driving_agent_state(std::string const &driving_agent_name) const override;

    void set_driving_agent_states(structures::IArray<IDrivingAgentState const*> *driving_agent_states) override;
    void set_driving_agent_state(IDrivingAgentState const *driving_agent_state) override;
};

}
}
}
