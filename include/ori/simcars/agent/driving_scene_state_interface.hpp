#pragma once

#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/agent/constant_interface.hpp>
#include <ori/simcars/agent/state_interface.hpp>
#include <ori/simcars/agent/driving_enums.hpp>
#include <ori/simcars/agent/driving_agent_state_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class IDrivingSceneState : public virtual IState
{
public:
    virtual std::shared_ptr<structures::IArray<std::shared_ptr<const IDrivingAgentState>>> get_driving_agent_states() const = 0;
    virtual std::shared_ptr<const IDrivingAgentState> get_driving_agent_state(const std::string& driving_agent_name) const = 0;

    virtual void set_driving_agent_states(
            std::shared_ptr<structures::IArray<std::shared_ptr<const IDrivingAgentState>>> driving_agent_states) = 0;
    virtual void set_driving_agent_state(std::shared_ptr<const IDrivingAgentState> driving_agent_state) = 0;
};

}
}
}
