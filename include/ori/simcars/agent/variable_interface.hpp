#pragma once

#include <ori/simcars/agent/event_interface.hpp>
#include <ori/simcars/agent/valueless_variable_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

template <typename T>
class IVariable : public virtual IValuelessVariable
{
public:
    virtual IVariable<T>* deep_copy() const = 0;

    virtual T const& get_value(temporal::Time time) const = 0;

    virtual structures::IArray<IEvent<T> const*>* get_events(
            temporal::Time time_window_start = temporal::Time::min(),
            temporal::Time time_window_end = temporal::Time::max()) const = 0;
    virtual IEvent<T> const* get_event(temporal::Time time, bool exact = false) const = 0;

    virtual void set_value(temporal::Time time, T const &value) = 0;

    virtual structures::IArray<IEvent<T>*>* get_mutable_events(
            temporal::Time time_window_start = temporal::Time::min(),
            temporal::Time time_window_end = temporal::Time::max()) = 0;
    virtual IEvent<T>* get_mutable_event(temporal::Time time, bool exact = false) = 0;
};

}
}
}
