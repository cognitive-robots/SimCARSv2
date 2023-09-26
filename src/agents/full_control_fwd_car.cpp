
#include <ori/simcars/agents/full_control_fwd_car.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

FullControlFWDCar::FullControlFWDCar(map::IMap const *map, FP_DATA_TYPE mass_value,
                                     FP_DATA_TYPE length_value, FP_DATA_TYPE width_value,
                                     FP_DATA_TYPE height_value, FP_DATA_TYPE wheel_radius_value,
                                     FP_DATA_TYPE axel_dist_value,
                                     FP_DATA_TYPE max_motor_torque_value,
                                     FP_DATA_TYPE min_motor_torque_value,
                                     FP_DATA_TYPE max_abs_steer_value, FP_DATA_TYPE drag_area_value,
                                     FP_DATA_TYPE cornering_stiffness_value) :
    MotorTorqueControlFWDCar(mass_value, length_value, width_value, height_value,
                             wheel_radius_value, axel_dist_value, max_motor_torque_value,
                             min_motor_torque_value, drag_area_value,
                             cornering_stiffness_value),
    SteerControlFWDCar(map, mass_value, length_value, width_value, height_value, wheel_radius_value,
                       axel_dist_value, max_abs_steer_value, drag_area_value,
                       cornering_stiffness_value),
    FWDCar(mass_value, length_value, width_value, height_value, wheel_radius_value, axel_dist_value,
           drag_area_value, cornering_stiffness_value)
{
}

}
}
}
