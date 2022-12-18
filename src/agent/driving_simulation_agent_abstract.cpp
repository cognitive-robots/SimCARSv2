
#include <ori/simcars/agent/driving_simulation_agent_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

IDrivingAgent* ADrivingSimulationAgent::driving_agent_deep_copy(IDrivingScene *driving_scene) const
{
    return this->driving_simulation_agent_deep_copy(dynamic_cast<IDrivingSimulationScene*>(driving_scene));
}

IDrivingScene const* ADrivingSimulationAgent::get_driving_scene() const
{
    return this->get_driving_simulation_scene();
}

}
}
}
