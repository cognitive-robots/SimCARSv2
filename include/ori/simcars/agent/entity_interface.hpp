#pragma once

#include <ori/simcars/structures/array_interface.hpp>
#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/agent/valueless_constant_interface.hpp>
#include <ori/simcars/agent/valueless_variable_interface.hpp>
#include <ori/simcars/agent/state_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class IEntity
{
public:
    virtual ~IEntity() = default;

    virtual std::shared_ptr<IEntity> entity_deep_copy() const = 0;

    virtual std::string get_name() const = 0;

    virtual geometry::Vec get_min_spatial_limits() const = 0;
    virtual geometry::Vec get_max_spatial_limits() const = 0;

    virtual temporal::Time get_min_temporal_limit() const = 0;
    virtual temporal::Time get_max_temporal_limit() const = 0;

    virtual std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessConstant>>> get_constant_parameters() const = 0;
    virtual std::shared_ptr<const IValuelessConstant> get_constant_parameter(const std::string& constant_name) const = 0;

    virtual std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessVariable>>> get_variable_parameters() const = 0;
    virtual std::shared_ptr<const IValuelessVariable> get_variable_parameter(const std::string& variable_name) const = 0;

    virtual std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessEvent>>> get_events() const = 0;

    virtual std::shared_ptr<IState> get_state(temporal::Time time, bool throw_on_out_of_range = true) const = 0;
};

}
}
}
