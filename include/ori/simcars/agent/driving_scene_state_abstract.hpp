#pragma once

#include <ori/simcars/agent/driving_scene_state_interface.hpp>
#include <ori/simcars/agent/read_only_driving_scene_state_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class ADrivingSceneState : public virtual AReadOnlyDrivingSceneState, public virtual IDrivingSceneState
{
    structures::IArray<IEntityState*>* get_mutable_entity_states() override;
    IEntityState* get_mutable_entity_state(std::string const &entity_name) override;
};

}
}
}
