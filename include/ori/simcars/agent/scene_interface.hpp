#pragma once

#include <ori/simcars/structures/array_interface.hpp>
#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/temporal/typedefs.hpp>
#include <ori/simcars/agent/valueless_constant_interface.hpp>
#include <ori/simcars/agent/valueless_event_interface.hpp>
#include <ori/simcars/agent/valueless_variable_interface.hpp>
#include <ori/simcars/agent/scene_state_interface.hpp>
#include <ori/simcars/agent/entity_interface.hpp>

#include <string>

namespace ori
{
namespace simcars
{
namespace agent
{

class IScene
{
public:
    virtual ~IScene() = default;

    virtual geometry::Vec get_min_spatial_limits() const = 0;
    virtual geometry::Vec get_max_spatial_limits() const = 0;

    virtual temporal::Time get_min_temporal_limit() const = 0;
    virtual temporal::Time get_max_temporal_limit() const = 0;

    virtual structures::IArray<IEntity const*>* get_entities() const = 0;
    virtual IEntity const* get_entity(std::string const &entity_name) const = 0;

    virtual IReadOnlySceneState const* get_state(temporal::Time time) const = 0;


    virtual structures::IArray<IEntity*>* get_mutable_entities() = 0;
    virtual IEntity* get_mutable_entity(std::string const &entity_name) = 0;

    virtual ISceneState* get_mutable_state(temporal::Time time) = 0;
};

}
}
}
