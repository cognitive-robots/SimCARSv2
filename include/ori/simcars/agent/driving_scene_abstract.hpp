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
    std::shared_ptr<IState> get_state(temporal::Time time) const override;

    std::shared_ptr<IDrivingSceneState> get_driving_scene_state(temporal::Time time) const override;
};

}
}
}
