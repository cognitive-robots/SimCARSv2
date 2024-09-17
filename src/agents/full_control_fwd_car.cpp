
#include <ori/simcars/agents/full_control_fwd_car.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

void FullControlFWDCar::init_links()
{
    MotorTorqueControlFWDCar::init_links();
    SteerControlFWDCar::init_links();
}

FullControlFWDCar::FullControlFWDCar(map::IDrivingMap const *map, FP_DATA_TYPE max_motor_torque_value,
                                     FP_DATA_TYPE min_motor_torque_value,
                                     FP_DATA_TYPE max_abs_steer_value) :
    MotorTorqueControlFWDCar(max_motor_torque_value, min_motor_torque_value),
    SteerControlFWDCar(map, max_abs_steer_value)
{
}

FullControlFWDCar::FullControlFWDCar(FullControlFWDCar const &control_fwd_car) :
    MotorTorqueControlFWDCar(control_fwd_car),
    SteerControlFWDCar(control_fwd_car)
{
}

}
}
}
