
#include <ori/simcars/agent/driving_simulation_scene_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

IDrivingScene* ADrivingSimulationScene::driving_scene_deep_copy() const
{
    return this->driving_simulation_scene_deep_copy();
}

ISimulationScene* ADrivingSimulationScene::simulation_scene_deep_copy() const
{
    return this->driving_simulation_scene_deep_copy();
}

}
}
}
