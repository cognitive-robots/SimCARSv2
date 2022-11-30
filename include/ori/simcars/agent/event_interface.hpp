#pragma once

#include <ori/simcars/agent/valueless_event_interface.hpp>
#include <ori/simcars/agent/constant_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

template <typename T>
class IEvent : public virtual IConstant<T>, public virtual IValuelessEvent
{
public:
    virtual IEvent<T>* event_shallow_copy() const = 0;
};

}
}
}
