#pragma once

#include <ori/simcars/temporal/typedefs.hpp>
#include <ori/simcars/agent/state_interface.hpp>

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

    virtual void simulate(std::shared_ptr<const agent::IState> current_state, std::shared_ptr<agent::IState> next_state, temporal::Duration time_step) const = 0;
};

}
}
}
