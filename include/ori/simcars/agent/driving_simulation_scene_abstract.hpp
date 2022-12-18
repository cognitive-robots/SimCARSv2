#pragma once

#include <ori/simcars/agent/driving_simulation_scene_interface.hpp>
#include <ori/simcars/agent/driving_scene_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class ADrivingSimulationScene : public virtual ADrivingScene, public virtual IDrivingSimulationScene
{
public:
    IDrivingScene* driving_scene_deep_copy() const override;
    ISimulationScene* simulation_scene_deep_copy() const override;
};

}
}
}
