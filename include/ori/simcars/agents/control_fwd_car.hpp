#pragma once

#include <ori/simcars/geometry/defines.hpp>
#include <ori/simcars/causal/endogenous_variable_interface.hpp>
#include <ori/simcars/causal/variable_types/exogenous/id_socket.hpp>
#include <ori/simcars/causal/variable_types/exogenous/scalar_socket.hpp>
#include <ori/simcars/causal/variable_types/exogenous/time_socket.hpp>
#include <ori/simcars/agents/declarations.hpp>
#include <ori/simcars/agents/plan_fwd_car.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class ControlFWDCar
{
protected:
    virtual void init_links();

    FWDCar *fwd_car;

    simcars::causal::TimeSocketVariable lon_lin_vel_time_goal;
    simcars::causal::ScalarSocketVariable lon_lin_vel_val_goal;
    simcars::causal::IdSocketVariable lane_val_goal;
    simcars::causal::TimeSocketVariable lane_time_goal;

    simcars::causal::ScalarSocketVariable motor_torque;
    simcars::causal::ScalarSocketVariable steer;

public:
    FWDCar* get_fwd_car();

    void set_fwd_car(FWDCar *fwd_car);

    friend void PlanFWDCar::set_control_fwd_car(ControlFWDCar *control_fwd_car);
};

}
}
}
