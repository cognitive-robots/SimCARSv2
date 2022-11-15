#pragma once

#include <ori/simcars/geometry/trig_buff.hpp>
#include <ori/simcars/map/map_interface.hpp>
#include <ori/simcars/agent/scene_interface.hpp>
#include <ori/simcars/agent/driving_agent_interface.hpp>
#include <ori/simcars/agent/driving_agent_controller_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class BasicDrivingAgentController : public virtual IDrivingAgentController
{
    geometry::TrigBuff const *trig_buff;

    map::IMap<std::string> const *map;

    temporal::Duration time_step;
    size_t steering_lookahead_steps;

public:
    BasicDrivingAgentController(map::IMap<std::string> const *map, temporal::Duration time_step, size_t steering_lookahead_steps);

    void modify_state(agent::IState const *original_state, agent::IState *modified_state) const override;

    void modify_driving_agent_state(agent::IDrivingAgentState const *original_state, agent::IDrivingAgentState *modified_state) const override;
};

}
}
}
