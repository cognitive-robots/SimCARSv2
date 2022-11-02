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
    virtual std::shared_ptr<structures::IArray<std::shared_ptr<const IDrivingAgent>>> get_driving_agents() const = 0;
    virtual std::shared_ptr<const IDrivingAgent> get_driving_agent(const std::string& driving_agent_name) const = 0;

    virtual std::shared_ptr<IDrivingSceneState> get_driving_scene_state(temporal::Time time) const = 0;
};

}
}
}
