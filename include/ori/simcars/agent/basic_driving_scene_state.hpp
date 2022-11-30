#pragma once

#include <ori/simcars/structures/stl/stl_dictionary.hpp>
#include <ori/simcars/agent/driving_agent_state_interface.hpp>
#include <ori/simcars/agent/driving_scene_state_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class BasicDrivingSceneState : public virtual IDrivingSceneState
{
protected:
    structures::stl::STLDictionary<std::string, IDrivingAgentState const*> driving_agent_state_dict;

public:
    BasicDrivingSceneState();
    BasicDrivingSceneState(IDrivingSceneState const *driving_scene_state);

    ~BasicDrivingSceneState();

    structures::IArray<IValuelessConstant const*>* get_parameter_values() const override;
    IValuelessConstant const* get_parameter_value(std::string const &parameter_name) const override;

    structures::IArray<IDrivingAgentState const*>* get_driving_agent_states() const override;
    IDrivingAgentState const* get_driving_agent_state(std::string const &driving_agent_name) const override;

    void set_parameter_values(structures::IArray<IValuelessConstant const*> const *parameter_values) override;
    void set_parameter_value(IValuelessConstant const *parameter_value) override;

    void set_driving_agent_states(structures::IArray<IDrivingAgentState const*> *driving_agent_states) override;
    void set_driving_agent_state(IDrivingAgentState const *driving_agent_state) override;
};

}
}
}
