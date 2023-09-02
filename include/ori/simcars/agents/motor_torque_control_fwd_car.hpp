#pragma once

#include <ori/simcars/causal/variable_types/exogenous/time_socket.hpp>
#include <ori/simcars/causal/variable_types/exogenous/duration_fixed.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_max.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_min.hpp>
#include <ori/simcars/causal/variable_types/endogenous/time_current_time_difference.hpp>
#include <ori/simcars/causal/variable_types/endogenous/duration_seconds_cast.hpp>
#include <ori/simcars/agents/fwd_car.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class MotorTorqueControlFWDCar : public FWDCar
{
protected:
    causal::ScalarFixedVariable max_motor_torque;
    causal::ScalarFixedVariable min_motor_torque;

    causal::TimeSocketVariable lon_lin_vel_time_goal;
    causal::ScalarSocketVariable lon_lin_vel_val_goal;

    causal::TimeCurrentTimeDifferenceVariable time_error;
    causal::DurationSecondsCastVariable time_error_secs;
    causal::ScalarFixedVariable min_act_horizon_secs;
    causal::ScalarMaxVariable actual_act_horizon_secs;
    causal::ScalarReciprocalVariable actual_act_horizon_secs_recip;

    causal::ScalarNegationVariable neg_lon_lin_vel;
    causal::ScalarSumVariable lon_lin_vel_error;

    causal::ScalarProductVariable needed_lon_lin_acc;
    causal::ScalarProductVariable needed_lon_force_plus_lon_env_force;
    causal::VectorDotProductVariable lon_env_force;
    causal::ScalarNegationVariable neg_lon_env_force;
    causal::ScalarSumVariable needed_lon_force;
    causal::ScalarProductVariable needed_motor_torque;

    causal::ScalarMinVariable max_lim_motor_torque;
    causal::ScalarMaxVariable actual_motor_torque;

public:
    MotorTorqueControlFWDCar(FP_DATA_TYPE mass_value, FP_DATA_TYPE length_value,
                             FP_DATA_TYPE width_value, FP_DATA_TYPE height_value,
                             FP_DATA_TYPE wheel_radius_value, FP_DATA_TYPE axel_distance_value,
                             FP_DATA_TYPE max_motor_torque_value,
                             FP_DATA_TYPE min_motor_torque_value,
                             FP_DATA_TYPE drag_area_value = 0.631,
                             FP_DATA_TYPE cornering_stiffness_value = 49675.0);
};

}
}
}
