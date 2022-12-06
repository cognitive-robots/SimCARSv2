#pragma once

#include <ori/simcars/agent/driving_scene_interface.hpp>
#include <ori/simcars/agent/driving_scene_state_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class ViewDrivingSceneState : public virtual ADrivingSceneState
{
    IDrivingScene *scene;
    temporal::Time time;

public:
    ViewDrivingSceneState(IDrivingScene *scene, temporal::Time time);

    temporal::Time get_time() const override;

    structures::IArray<IReadOnlyDrivingAgentState const*>* get_driving_agent_states() const override;
    IReadOnlyDrivingAgentState const* get_driving_agent_state(std::string const &driving_agent_name) const override;

    IDrivingScene const* get_scene() const;


    structures::IArray<IDrivingAgentState*>* get_mutable_driving_agent_states() override;
    IDrivingAgentState* get_mutable_driving_agent_state(std::string const &driving_agent_name) override;

    void set_scene(IDrivingScene *scene);
    void set_time(temporal::Time time);
};

}
}
}
