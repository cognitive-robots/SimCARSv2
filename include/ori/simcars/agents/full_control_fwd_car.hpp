#pragma once

#include <ori/simcars/causal/variable_types/exogenous/id_socket.hpp>
#include <ori/simcars/causal/variable_types/endogenous/id_proxy.hpp>
#include <ori/simcars/causal/variable_types/endogenous/lane_map_point.hpp>
#include <ori/simcars/agents/motor_torque_control_fwd_car.hpp>
#include <ori/simcars/agents/steer_control_fwd_car.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class FullControlFWDCar : public virtual MotorTorqueControlFWDCar, public virtual SteerControlFWDCar
{
public:
    FullControlFWDCar(map::IMap const *map, FP_DATA_TYPE mass_value, FP_DATA_TYPE length_value,
                      FP_DATA_TYPE width_value, FP_DATA_TYPE height_value,
                      FP_DATA_TYPE wheel_radius_value, FP_DATA_TYPE axel_dist_value,
                      FP_DATA_TYPE max_motor_torque_value, FP_DATA_TYPE min_motor_torque_value,
                      FP_DATA_TYPE max_abs_steer_value, FP_DATA_TYPE drag_area_value = 0.631,
                      FP_DATA_TYPE cornering_stiffness_value = 49675.0);
};

}
}
}
