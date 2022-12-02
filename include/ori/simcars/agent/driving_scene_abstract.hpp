#pragma once

#include <ori/simcars/agent/driving_scene_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class ADrivingScene : public virtual IDrivingScene
{
public:
    structures::IArray<IEntity const*>* get_entities() const override;
    IEntity const* get_entity(std::string const &entity_name) const override;

    IReadOnlySceneState const* get_state(temporal::Time time) const override;

    IReadOnlyDrivingSceneState const* get_driving_scene_state(temporal::Time time) const override;


    structures::IArray<IEntity*>* get_mutable_entities() override;
    IEntity* get_mutable_entity(std::string const &entity_name) override;

    ISceneState* get_mutable_state(temporal::Time time) override;

    IDrivingSceneState* get_mutable_driving_scene_state(temporal::Time time) override;
};

}
}
}
