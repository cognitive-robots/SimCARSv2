#pragma once

#include <ori/simcars/agent/read_only_driving_scene_state_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class AReadOnlyDrivingSceneState : public virtual IReadOnlyDrivingSceneState
{
    structures::IArray<IReadOnlyEntityState const*>* get_entity_states() const override;
    IReadOnlyEntityState const* get_entity_state(std::string const &entity_name) const override;
};

}
}
}
