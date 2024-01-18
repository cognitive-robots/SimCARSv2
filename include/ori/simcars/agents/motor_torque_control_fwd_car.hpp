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

    simcars::causal::ScalarFixedVariable max_motor_torque;
    simcars::causal::ScalarFixedVariable min_motor_torque;

    simcars::causal::TimeCurrentTimeDifferenceVariable time_error;
    simcars::causal::DurationSecondsCastVariable time_error_secs;
    simcars::causal::ScalarFixedVariable min_act_horizon_secs;
    simcars::causal::ScalarMaxVariable actual_act_horizon_secs;
    simcars::causal::ScalarReciprocalVariable actual_act_horizon_secs_recip;

    simcars::causal::ScalarSocketVariable speed;
    simcars::causal::ScalarNegationVariable neg_speed;
    simcars::causal::ScalarSumVariable speed_error;

    simcars::causal::ScalarSocketVariable mass;
    simcars::causal::ScalarSocketVariable wheel_radius;
    simcars::causal::VectorSocketVariable dir;
    simcars::causal::VectorProxyVariable dir_proxy;
    simcars::causal::VectorSocketVariable env_force;
    simcars::causal::ScalarProductVariable needed_acc;
    simcars::causal::ScalarProductVariable needed_force_plus_lon_env_force;
    simcars::causal::VectorDotProductVariable lon_env_force;
    simcars::causal::ScalarNegationVariable neg_lon_env_force;
    simcars::causal::ScalarSumVariable needed_force;
    simcars::causal::ScalarProductVariable needed_motor_torque;

    simcars::causal::ScalarMinVariable max_lim_motor_torque;
    simcars::causal::ScalarMaxVariable actual_motor_torque;

public:
    MotorTorqueControlFWDCar(FP_DATA_TYPE max_motor_torque_value,
                             FP_DATA_TYPE min_motor_torque_value);
    MotorTorqueControlFWDCar(MotorTorqueControlFWDCar const &control_fwd_car);
};

}
}
}
