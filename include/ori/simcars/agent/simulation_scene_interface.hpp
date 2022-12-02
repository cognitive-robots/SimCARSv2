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
    virtual inline temporal::Duration get_time_step() const = 0;

    virtual void simulate(temporal::Time time) = 0;
};

}
}
}
