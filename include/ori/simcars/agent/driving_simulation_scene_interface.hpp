#pragma once

#include <ori/simcars/agent/driving_scene_interface.hpp>
#include <ori/simcars/agent/simulation_scene_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class IDrivingSimulationScene : public virtual IDrivingScene, public virtual ISimulationScene
{
public:
    virtual IDrivingSimulationScene* driving_simulation_scene_deep_copy() const = 0;
};

}
}
}
