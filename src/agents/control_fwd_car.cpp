
#include <ori/simcars/agents/control_fwd_car.hpp>

#include <ori/simcars/agents/fwd_car.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

void ControlFWDCar::set_motor_torque_control(
        causal::IEndogenousVariable<FP_DATA_TYPE> *motor_torque)
{
    fwd_car->motor_torque.set_parent(motor_torque);
}

void ControlFWDCar::set_steer_control(causal::IEndogenousVariable<FP_DATA_TYPE> *steer)
{
    fwd_car->steer.set_parent(steer);
}

void ControlFWDCar::set_fwd_car(FWDCar *fwd_car)
{
    this->fwd_car = fwd_car;
}

}
}
}
