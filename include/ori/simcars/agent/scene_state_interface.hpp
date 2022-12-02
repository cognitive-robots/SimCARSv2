#pragma once

#include <ori/simcars/agent/read_only_scene_state_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class ISceneState : public virtual IReadOnlySceneState
{
public:
    virtual structures::IArray<IEntityState*>* get_mutable_entity_states() = 0;
    virtual IEntityState* get_mutable_entity_state(std::string const &entity_name) = 0;
};

}
}
}
