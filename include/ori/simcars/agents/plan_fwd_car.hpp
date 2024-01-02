#pragma once

#include <ori/simcars/causal/variable_types/exogenous/id_socket.hpp>
#include <ori/simcars/causal/variable_types/exogenous/scalar_socket.hpp>
#include <ori/simcars/causal/variable_types/exogenous/time_socket.hpp>
#include <ori/simcars/agents/declarations.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class PlanFWDCar
{
protected:
    virtual void init_links();

    ControlFWDCar *control_fwd_car;

    causal::ScalarSocketVariable speed_val_goal;
    causal::TimeSocketVariable speed_time_goal;
    causal::IdSocketVariable lane_val_goal;
    causal::TimeSocketVariable lane_time_goal;

public:
    ControlFWDCar* get_control_fwd_car();

    void set_control_fwd_car(ControlFWDCar *control_fwd_car);
};

}
}
}
