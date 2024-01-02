
#include <ori/simcars/agents/plan_fwd_car.hpp>

#include <ori/simcars/agents/control_fwd_car.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

ControlFWDCar* PlanFWDCar::get_control_fwd_car()
{
    return control_fwd_car;
}

void PlanFWDCar::set_control_fwd_car(ControlFWDCar *control_fwd_car)
{
    this->control_fwd_car = control_fwd_car;

    control_fwd_car->lon_lin_vel_val_goal.set_parent(&speed_val_goal);
    control_fwd_car->lon_lin_vel_time_goal.set_parent(&speed_time_goal);
    control_fwd_car->lane_val_goal.set_parent(&lane_val_goal);
    control_fwd_car->lane_time_goal.set_parent(&lane_time_goal);

    init_links();
}

}
}
}
