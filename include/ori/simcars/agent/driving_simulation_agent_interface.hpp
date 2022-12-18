#pragma once

#include <ori/simcars/agent/declarations.hpp>
#include <ori/simcars/agent/driving_agent_interface.hpp>
#include <ori/simcars/agent/driving_simulation_scene_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class IDrivingSimulationAgent : public virtual IDrivingAgent
{
public:
    virtual IDrivingSimulationAgent* driving_simulation_agent_deep_copy(IDrivingSimulationScene *driving_simulation_scene = nullptr) const = 0;

    virtual IDrivingSimulationScene const* get_driving_simulation_scene() const = 0;

    virtual void begin_simulation(temporal::Time simulation_start_time) const = 0;
};

}
}
}
