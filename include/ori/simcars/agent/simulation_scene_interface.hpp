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
    virtual void simulate(temporal::Time time) = 0;
};

}
}
}
