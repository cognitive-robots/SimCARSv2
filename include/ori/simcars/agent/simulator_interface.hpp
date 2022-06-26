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

class ISimulator
{
public:
    virtual ~ISimulator() = default;

    virtual std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessEvent>>> simulate(temporal::Time current_time, temporal::Duration time_step) const = 0;
};

}
}
}
