
#include <ori/simcars/agents/control_fwd_car.hpp>

#include <ori/simcars/agents/fwd_car.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

ControlFWDCar::ControlFWDCar() :
    fwd_car(nullptr),

    action(),
    action_buff(&action, nullptr, true),

    speed_goal(&action_buff),
    speed_val_goal(&speed_goal),
    speed_time_goal(&speed_goal),

    lane_goal(&action_buff),
    lane_val_goal(&lane_goal),
    lane_time_goal(&lane_goal),

    motor_torque(),
    steer()
{
}

FWDCar* ControlFWDCar::get_fwd_car()
{
    return fwd_car;
}

void ControlFWDCar::set_fwd_car(FWDCar *fwd_car)
{
    if (this->fwd_car != nullptr)
    {
        this->fwd_car->motor_torque.set_parent(nullptr);
        this->fwd_car->motor_torque_buff.set_axiomatic(true);

        this->fwd_car->steer.set_parent(nullptr);
        this->fwd_car->steer_buff.set_axiomatic(true);
    }
    if (fwd_car != nullptr)
    {
        this->fwd_car = fwd_car;

        fwd_car->motor_torque.set_parent(&motor_torque);
        fwd_car->motor_torque_buff.set_axiomatic(false);

        fwd_car->steer.set_parent(&steer);
        fwd_car->steer_buff.set_axiomatic(false);

        init_links();
    }
}

}
}
}
