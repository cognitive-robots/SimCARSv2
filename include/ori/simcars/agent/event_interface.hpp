#pragma once

#include <ori/simcars/agent/valueless_event_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

template <typename T>
class IEvent : public virtual IValuelessEvent
{
public:
    virtual IEvent<T>* shallow_copy() const = 0;

    virtual T get_value() const = 0;
};

}
}
}
