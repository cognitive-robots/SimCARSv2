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
    structures::stl::STLDictionary<std::string, std::shared_ptr<const IDrivingAgentState>> driving_agent_state_dict;

public:
    BasicDrivingSceneState();
    BasicDrivingSceneState(std::shared_ptr<const IDrivingSceneState> driving_scene_state);

    std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessConstant>>> get_parameter_values() const override;
    std::shared_ptr<const IValuelessConstant> get_parameter_value(const std::string& parameter_name) const override;

    std::shared_ptr<structures::IArray<std::shared_ptr<const IDrivingAgentState>>> get_driving_agent_states() const override;
    std::shared_ptr<const IDrivingAgentState> get_driving_agent_state(const std::string& driving_agent_name) const override;

    void set_driving_agent_states(
                std::shared_ptr<structures::IArray<std::shared_ptr<const IDrivingAgentState>>> driving_agent_states) override;
    void set_driving_agent_state(std::shared_ptr<const IDrivingAgentState> driving_agent_state) override;
};

}
}
}
