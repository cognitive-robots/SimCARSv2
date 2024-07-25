#pragma once

#include <ori/simcars/agents/declarations.hpp>
#include <ori/simcars/agents/causal/variable_types/exogenous/ped_action_socket.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class PlanPed
{
protected:
    virtual void init_links() = 0;

    ControlPed *control_ped;

    causal::PedActionSocketVariable action;

public:
    PlanPed();

    ControlPed* get_control_ped();

    void set_control_ped(ControlPed *control_ped);
};

}
}
}
