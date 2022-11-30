#pragma once

#include <ori/simcars/temporal/typedefs.hpp>
#include <ori/simcars/agent/valueless_constant_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class IValuelessEvent : public virtual IValuelessConstant
{
public:
    virtual ~IValuelessEvent() = default;

    virtual IValuelessEvent* valueless_event_shallow_copy() const = 0;

    virtual temporal::Time get_time() const = 0;
};

}
}
}
