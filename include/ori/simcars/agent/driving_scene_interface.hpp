#pragma once

#include <ori/simcars/agent/driving_agent_interface.hpp>
#include <ori/simcars/agent/driving_scene_state_interface.hpp>
#include <ori/simcars/agent/scene_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class IDrivingScene : public virtual IScene
{
public:
    virtual structures::IArray<IDrivingAgent const*>* get_driving_agents() const = 0;
    virtual IDrivingAgent const* get_driving_agent(std::string const &driving_agent_name) const = 0;

    virtual IReadOnlyDrivingSceneState const* get_driving_scene_state(temporal::Time time) const = 0;


    virtual structures::IArray<IDrivingAgent*>* get_mutable_driving_agents() = 0;
    virtual IDrivingAgent* get_mutable_driving_agent(std::string const &driving_agent_name) = 0;

    virtual IDrivingSceneState* get_mutable_driving_scene_state(temporal::Time time) = 0;
};

}
}
}
