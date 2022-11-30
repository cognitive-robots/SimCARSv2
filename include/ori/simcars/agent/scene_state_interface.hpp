#pragma once

#include <ori/simcars/agent/entity_state_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class ISceneState
{
public:
    virtual ~ISceneState() = default;

    virtual structures::IArray<IEntityState const*>* get_entity_states() const = 0;
    virtual IEntityState const* get_entity_state(std::string const &entity_name) const = 0;

    virtual void set_entity_states(structures::IArray<IEntityState const*> const *entities) = 0;
    virtual void set_entity_state(IEntityState const *entity) = 0;
};

}
}
}
