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
protected:
    void init_links() override;

public:
    FullControlFWDCar(map::IMap const *map, FP_DATA_TYPE max_motor_torque_value,
                      FP_DATA_TYPE min_motor_torque_value, FP_DATA_TYPE max_abs_steer_value);
};

}
}
}
