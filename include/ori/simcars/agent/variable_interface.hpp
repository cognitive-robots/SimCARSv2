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
class IVariable : public IValuelessVariable
{
public:
    virtual std::shared_ptr<IVariable<T>> deep_copy() const = 0;

    virtual T get_value(temporal::Time time) const = 0;

    virtual std::shared_ptr<structures::IArray<std::shared_ptr<const IEvent<T>>>> get_events() const = 0;
    virtual std::shared_ptr<const IEvent<T>> get_event(temporal::Time time) const = 0;
    virtual void add_event(std::shared_ptr<const IEvent<T>> event) = 0;
    virtual bool remove_event(std::shared_ptr<const IEvent<T>> event) = 0;
};

}
}
}
