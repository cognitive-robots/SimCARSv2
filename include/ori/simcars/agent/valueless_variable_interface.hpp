#pragma once

#include <ori/simcars/structures/array_interface.hpp>
#include <ori/simcars/temporal/typedefs.hpp>
#include <ori/simcars/agent/valueless_event_interface.hpp>

#include <string>
#include <functional>

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
        GOAL = 3,
        EXTERNAL = 4
    };

    virtual ~IValuelessVariable() = default;

    virtual IValuelessVariable* valueless_deep_copy() const = 0;

    virtual std::string get_full_name() const = 0;
    virtual std::string get_entity_name() const = 0;
    virtual std::string get_parameter_name() const = 0;
    virtual std::string get_variable_name() const = 0;
    virtual std::string get_type_name() const = 0;

    virtual Type get_type() const = 0;

    virtual bool get_value_as_string(temporal::Time time, std::string &str) const = 0;

    virtual temporal::Time get_min_temporal_limit() const = 0;
    virtual temporal::Time get_last_event_time() const = 0;
    virtual temporal::Time get_max_temporal_limit() const = 0;

    virtual bool has_event(temporal::Time time) const = 0;

    virtual structures::IArray<IValuelessEvent const*>* get_valueless_events(
            temporal::Time time_window_start = temporal::Time::min(),
            temporal::Time time_window_end = temporal::Time::max()) const = 0;
    virtual IValuelessEvent const* get_valueless_event(temporal::Time time, bool exact = false) const = 0;

    virtual void propogate_events_forward() const = 0;
    virtual void propogate_events_forward(temporal::Time time_window_end) const = 0;

    virtual bool remove_value(temporal::Time time) = 0;

    virtual structures::IArray<IValuelessEvent*>* get_mutable_valueless_events(
            temporal::Time time_window_start = temporal::Time::min(),
            temporal::Time time_window_end = temporal::Time::max()) = 0;
    virtual IValuelessEvent* get_mutable_valueless_event(temporal::Time time, bool exact = false) = 0;
};

}
}
}
