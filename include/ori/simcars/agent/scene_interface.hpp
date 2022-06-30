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
#include <memory>

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

    virtual std::shared_ptr<structures::IArray<std::shared_ptr<const IEntity>>> get_entities() const = 0;
    virtual std::shared_ptr<const IEntity> get_entity(const std::string& entity_name) const = 0;

    virtual std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessConstant>>> get_constants() const = 0;
    virtual std::shared_ptr<const IValuelessConstant> get_constant(const std::string& constant_name) const = 0;

    virtual std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessVariable>>> get_variables() const = 0;
    virtual std::shared_ptr<const IValuelessVariable> get_variable(const std::string& variable_name) const = 0;

    virtual std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessEvent>>> get_events() const = 0;

    virtual std::shared_ptr<const IState> get_state(temporal::Time time) const = 0;
};

}
}
}
