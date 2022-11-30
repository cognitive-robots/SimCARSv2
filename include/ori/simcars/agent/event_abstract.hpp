#pragma once

#include <ori/simcars/agent/event_interface.hpp>
#include <ori/simcars/agent/constant_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

template <typename T>
class AEvent : public virtual AConstant<T>, public virtual IEvent<T>
{
public:
    IValuelessEvent* valueless_event_shallow_copy() const override
    {
        return this->event_shallow_copy();
    }
};

}
}
}
