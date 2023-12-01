#pragma once

#include <ori/simcars/causal/variable_types/exogenous/scalar_fixed.hpp>
#include <ori/simcars/causal/variable_types/exogenous/vector_socket.hpp>
#include <ori/simcars/causal/variable_types/exogenous/duration_fixed.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_max.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_min.hpp>
#include <ori/simcars/causal/variable_types/endogenous/time_current_time_difference.hpp>
#include <ori/simcars/causal/variable_types/endogenous/duration_seconds_cast.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_negation.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_reciprocal.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_sum.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_product.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_proxy.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_dot_product.hpp>
#include <ori/simcars/agents/control_fwd_car.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class MotorTorqueControlFWDCar : public virtual ControlFWDCar
{
protected:
    void init_links() override;

    causal::ScalarFixedVariable max_motor_torque;
    causal::ScalarFixedVariable min_motor_torque;

    causal::TimeCurrentTimeDifferenceVariable time_error;
    causal::DurationSecondsCastVariable time_error_secs;
    causal::ScalarFixedVariable min_act_horizon_secs;
    causal::ScalarMaxVariable actual_act_horizon_secs;
    causal::ScalarReciprocalVariable actual_act_horizon_secs_recip;

    causal::ScalarSocketVariable lon_lin_vel;
    causal::ScalarNegationVariable neg_lon_lin_vel;
    causal::ScalarSumVariable lon_lin_vel_error;

    causal::ScalarSocketVariable mass;
    causal::ScalarSocketVariable wheel_radius;
    causal::VectorSocketVariable dir;
    causal::VectorProxyVariable dir_proxy;
    causal::VectorSocketVariable env_force;
    causal::ScalarProductVariable needed_lon_lin_acc;
    causal::ScalarProductVariable needed_lon_force_plus_lon_env_force;
    causal::VectorDotProductVariable lon_env_force;
    causal::ScalarNegationVariable neg_lon_env_force;
    causal::ScalarSumVariable needed_lon_force;
    causal::ScalarProductVariable needed_motor_torque;

    causal::ScalarMinVariable max_lim_motor_torque;
    causal::ScalarMaxVariable actual_motor_torque;

public:
    MotorTorqueControlFWDCar(FP_DATA_TYPE max_motor_torque_value,
                             FP_DATA_TYPE min_motor_torque_value);
};

}
}
}
