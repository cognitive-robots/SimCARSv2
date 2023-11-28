#pragma once

#include <ori/simcars/geometry/defines.hpp>
#include <ori/simcars/causal/endogenous_variable_interface.hpp>
#include <ori/simcars/agents/declarations.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class ControlFWDCar
{
protected:
    FWDCar *fwd_car;

public:
    void set_motor_torque_control(causal::IEndogenousVariable<FP_DATA_TYPE> *motor_torque);
    void set_steer_control(causal::IEndogenousVariable<FP_DATA_TYPE> *steer);

    virtual void set_fwd_car(FWDCar *fwd_car);
};

}
}
}
