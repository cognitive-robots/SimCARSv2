#pragma once

#include <ori/simcars/agent/driving_scene_state_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class ADrivingSceneState : public virtual IDrivingSceneState
{
    structures::IArray<IEntityState const*>* get_entity_states() const override;
    IEntityState const* get_entity_state(std::string const &entity_name) const override;

    void set_entity_states(structures::IArray<IEntityState const*> const *entities) override;
    void set_entity_state(IEntityState const *entity) override;
};

}
}
}
