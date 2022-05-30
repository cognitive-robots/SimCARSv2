#pragma once

#include <ori/simcars/structures/array_interface.hpp>
#include <ori/simcars/temporal/typedefs.hpp>
#include <ori/simcars/agent/valueless_event_interface.hpp>

#include <string>
#include <memory>

namespace ori
{
namespace simcars
{
namespace agent
{

class IValuelessVariable
{
public:
    enum class Type : uint8_t
    {
        BASE = 0,
        INDIRECT_ACTUATION = 1,
        DIRECT_ACTUATION = 2,
        GOAL_VALUE = 3,
        GOAL_DURATION = 4
    };

    virtual ~IValuelessVariable() = default;

    virtual std::shared_ptr<IValuelessVariable> valueless_deep_copy() const = 0;

    virtual std::string get_full_name() const = 0;
    virtual std::string get_entity_name() const = 0;
    virtual std::string get_parameter_name() const = 0;
    virtual std::string get_type_name() const = 0;

    virtual Type get_type() const = 0;

    virtual std::string get_value_as_string(temporal::Time time) const = 0;

    virtual std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessEvent>>> get_valueless_events() const = 0;
    virtual std::shared_ptr<const IValuelessEvent> get_valueless_event(temporal::Time time) const = 0;
};

}
}
}
