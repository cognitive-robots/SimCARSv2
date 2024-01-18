#pragma once

#include <ori/simcars/agents/declarations.hpp>
#include <ori/simcars/agents/causal/variable_types/exogenous/fwd_car_action_socket.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class PlanFWDCar
{
protected:
    virtual void init_links() = 0;

    ControlFWDCar *control_fwd_car;

    causal::FWDCarActionSocketVariable action;

public:
    PlanFWDCar();

    ControlFWDCar* get_control_fwd_car();

    void set_control_fwd_car(ControlFWDCar *control_fwd_car);
};

}
}
}
