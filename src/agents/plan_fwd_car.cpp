
#include <ori/simcars/agents/plan_fwd_car.hpp>

#include <ori/simcars/agents/control_fwd_car.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

PlanFWDCar::PlanFWDCar() :
    control_fwd_car(nullptr),

    action()
{
}

ControlFWDCar* PlanFWDCar::get_control_fwd_car()
{
    return control_fwd_car;
}

void PlanFWDCar::set_control_fwd_car(ControlFWDCar *control_fwd_car)
{
    this->control_fwd_car = control_fwd_car;

    control_fwd_car->action.set_parent(&action);

    init_links();
}

}
}
}
