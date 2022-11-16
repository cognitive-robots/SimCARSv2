#pragma once

#include <ori/simcars/agent/driving_scene_interface.hpp>
#include <ori/simcars/agent/scene_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class ADrivingScene : public virtual AScene, public virtual IDrivingScene
{
public:
    IState* get_state(temporal::Time time) const override;

    IDrivingSceneState* get_driving_scene_state(temporal::Time time) const override;
};

}
}
}
