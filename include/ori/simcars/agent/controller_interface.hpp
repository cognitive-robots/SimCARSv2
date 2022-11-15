#pragma once

#include <ori/simcars/structures/array_interface.hpp>
#include <ori/simcars/temporal/typedefs.hpp>
#include <ori/simcars/agent/state_interface.hpp>

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

    virtual void modify_state(agent::IState const *original_state, agent::IState *modified_state) const = 0;
};

}
}
}
