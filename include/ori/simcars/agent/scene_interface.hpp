#pragma once

#include <ori/simcars/structures/array_interface.hpp>
#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/temporal/typedefs.hpp>
#include <ori/simcars/agent/valueless_constant_interface.hpp>
#include <ori/simcars/agent/valueless_event_interface.hpp>
#include <ori/simcars/agent/valueless_variable_interface.hpp>
#include <ori/simcars/agent/state_interface.hpp>
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

    virtual structures::IArray<IValuelessConstant const*>* get_constants() const = 0;
    virtual IValuelessConstant const* get_constant(std::string const &constant_name) const = 0;

    virtual structures::IArray<IValuelessVariable const*>* get_variables() const = 0;
    virtual IValuelessVariable const* get_variable(std::string const &variable_name) const = 0;

    virtual structures::IArray<IValuelessEvent const*>* get_events() const = 0;

    virtual IState* get_state(temporal::Time time) const = 0;
};

}
}
}
