#pragma once

#include <ori/simcars/agent/driving_simulation_agent_interface.hpp>
#include <ori/simcars/agent/driving_agent_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class ADrivingSimulationAgent : public virtual ADrivingAgent, public virtual IDrivingSimulationAgent
{
public:
    IDrivingAgent* driving_agent_deep_copy(IDrivingScene *driving_scene) const override;

    IDrivingScene const* get_driving_scene() const override;
};

}
}
}
