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
    std::shared_ptr<const geometry::TrigBuff> trig_buff;

    std::shared_ptr<const map::IMap<std::string>> map;

    temporal::Duration time_step;
    size_t steering_lookahead_steps;

public:
    BasicDrivingAgentController(std::shared_ptr<const map::IMap<std::string>> map, temporal::Duration time_step, size_t steering_lookahead_steps);

    void modify_state(std::shared_ptr<const agent::IState> original_state, std::shared_ptr<agent::IState> modified_state) const override;

    void modify_driving_agent_state(std::shared_ptr<const agent::IDrivingAgentState> original_state, std::shared_ptr<agent::IDrivingAgentState> modified_state) const override;
};

}
}
}
