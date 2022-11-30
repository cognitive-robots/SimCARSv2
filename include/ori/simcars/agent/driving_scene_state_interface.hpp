#pragma once

#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/agent/constant_interface.hpp>
#include <ori/simcars/agent/scene_state_interface.hpp>
#include <ori/simcars/agent/driving_enums.hpp>
#include <ori/simcars/agent/driving_agent_state_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class IDrivingSceneState : public virtual ISceneState
{
public:
    virtual structures::IArray<IDrivingAgentState const*>* get_driving_agent_states() const = 0;
    virtual IDrivingAgentState const* get_driving_agent_state(std::string const &driving_agent_name) const = 0;

    virtual void set_driving_agent_states(structures::IArray<IDrivingAgentState const*> *driving_agent_states) = 0;
    virtual void set_driving_agent_state(IDrivingAgentState const *driving_agent_state) = 0;
};

}
}
}
