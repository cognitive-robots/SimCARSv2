#pragma once

#include <ori/simcars/causal/variable_types/endogenous/id_proxy.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_proxy.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_buffer.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_previous_time_step.hpp>
#include <ori/simcars/causal/variable_types/endogenous/lane_map_point.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_negation.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_sum.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_scalar_product.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_cross_product.hpp>
#include <ori/simcars/agents/motor_torque_control_fwd_car.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class SteerControlFWDCar : public virtual ControlFWDCar
{
protected:
    void init_links() override;

    map::IDrivingMap const *map;

    simcars::causal::ScalarFixedVariable max_steer;
    simcars::causal::ScalarNegationVariable min_steer;

    simcars::causal::TimeCurrentTimeDifferenceVariable time_diff;
    simcars::causal::DurationSecondsCastVariable time_diff_secs;
    simcars::causal::ScalarFixedVariable min_act_horizon_secs;
    simcars::causal::ScalarMaxVariable actual_act_horizon_secs;
    simcars::causal::ScalarReciprocalVariable actual_act_horizon_secs_recip;

    simcars::causal::VectorSocketVariable pos;
    simcars::causal::VectorProxyVariable pos_proxy;
    simcars::causal::VectorSocketVariable lin_vel;
    simcars::causal::VectorProxyVariable lin_vel_proxy;
    simcars::causal::VectorScalarProductVariable act_horizon_expected_pos_diff;
    simcars::causal::VectorSumVariable act_horizon_expected_pos;
    simcars::causal::LaneMapPointVariable act_horizon_target_pos;
    simcars::causal::VectorNegationVariable neg_pos;
    simcars::causal::VectorSumVariable act_horizon_target_pos_diff;

    simcars::causal::VectorCrossProductVariable act_horizon_ang_sin;
    simcars::causal::VectorDotProductVariable act_horizon_ang_cos;
    simcars::causal::ScalarReciprocalVariable act_horizon_ang_cos_recip;
    simcars::causal::ScalarProductVariable act_horizon_ang;

    simcars::causal::ScalarNegationVariable ang_diff;
    simcars::causal::ScalarBufferVariable ang_diff_buff;
    simcars::causal::ScalarPreviousTimeStepVariable prev_ang_diff;
    simcars::causal::ScalarNegationVariable neg_prev_ang_diff;
    simcars::causal::ScalarSumVariable ang_diff_diff;

    simcars::causal::ScalarFixedVariable k_p;
    simcars::causal::ScalarProductVariable p_factor;
    simcars::causal::ScalarFixedVariable k_d;
    simcars::causal::ScalarProductVariable d_factor;
    simcars::causal::ScalarSumVariable needed_steer;

    /*
    simcars::causal::ScalarSocketVariable axle_dist;
    simcars::causal::ScalarFixedVariable double_scale_factor;
    simcars::causal::ScalarProxyVariable double_scale_factor_proxy;
    simcars::causal::ScalarProductVariable double_axle_dist;

    simcars::causal::ScalarSocketVariable lon_lin_vel_recip;
    simcars::causal::ScalarProductVariable needed_ang_vel_double_axle_dist_prod;
    simcars::causal::ScalarProductVariable needed_steer;
    */

    simcars::causal::ScalarMinVariable max_lim_steer;
    simcars::causal::ScalarMaxVariable actual_steer;

public:
    SteerControlFWDCar(map::IDrivingMap const *map, FP_DATA_TYPE max_abs_steer_value);
    SteerControlFWDCar(SteerControlFWDCar const &control_fwd_car);
};

}
}
}
