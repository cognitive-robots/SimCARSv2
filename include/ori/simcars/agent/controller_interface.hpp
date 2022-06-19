#pragma once

#include <ori/simcars/structures/array_interface.hpp>
#include <ori/simcars/temporal/typedefs.hpp>
#include <ori/simcars/agent/valueless_event_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class IController
{
public:
    virtual ~IController() = default;

    virtual std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessEvent>>> get_indirect_actuation_events(temporal::Time time) const = 0;
};

}
}
}
