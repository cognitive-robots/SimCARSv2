#pragma once

#include <ori/simcars/agent/scene_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class ISimulationScene : public virtual IScene
{
public:
    virtual ISimulationScene* simulation_scene_deep_copy() const = 0;

    virtual void simulate(temporal::Time time) = 0;
};

}
}
}
