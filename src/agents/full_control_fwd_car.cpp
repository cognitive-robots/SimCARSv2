
#include <ori/simcars/agents/full_control_fwd_car.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

FullControlFWDCar::FullControlFWDCar(map::IMap const *map, FP_DATA_TYPE max_motor_torque_value,
                                     FP_DATA_TYPE min_motor_torque_value,
                                     FP_DATA_TYPE max_abs_steer_value) :
    MotorTorqueControlFWDCar(max_motor_torque_value, min_motor_torque_value),
    SteerControlFWDCar(map, max_abs_steer_value)
{
}

void FullControlFWDCar::set_fwd_car(FWDCar *fwd_car)
{
    MotorTorqueControlFWDCar::set_fwd_car(fwd_car);
    SteerControlFWDCar::set_fwd_car(fwd_car);
}

}
}
}
