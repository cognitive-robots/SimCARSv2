
#include <ori/simcars/agents/control_fwd_car.hpp>

#include <ori/simcars/agents/fwd_car.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

FWDCar* ControlFWDCar::get_fwd_car()
{
    return fwd_car;
}

void ControlFWDCar::set_fwd_car(FWDCar *fwd_car)
{
    this->fwd_car = fwd_car;

    fwd_car->motor_torque.set_parent(&motor_torque);
    fwd_car->steer.set_parent(&steer);

    init_links();
}

}
}
}
