#pragma once

#include <ori/simcars/causal/variable_types/exogenous/id_socket.hpp>
#include <ori/simcars/causal/variable_types/endogenous/id_proxy.hpp>
#include <ori/simcars/causal/variable_types/endogenous/lane_map_point.hpp>
#include <ori/simcars/agents/motor_torque_control_fwd_car.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class FullControlFWDCar : public MotorTorqueControlFWDCar
{
protected:
    causal::ScalarFixedVariable max_steer;
    causal::ScalarNegationVariable min_steer;

    causal::TimeSocketVariable lane_time_goal;
    causal::IdSocketVariable lane_val_goal;
    causal::IdProxyVariable lane_val_goal_proxy;

    causal::TimeCurrentTimeDifferenceVariable time_error;
    causal::DurationSecondsCastVariable time_error_secs;
    causal::ScalarFixedVariable min_act_horizon_secs;
    causal::ScalarMaxVariable actual_act_horizon_secs;
    causal::ScalarReciprocalVariable actual_act_horizon_secs_recip;

    causal::VectorScalarProductVariable act_horizon_expected_pos_diff;
    causal::VectorSumVariable act_horizon_expected_pos;
    causal::LaneMapPointVariable act_horizon_target_pos;
    causal::VectorNegationVariable neg_pos;
    causal::VectorSumVariable act_horizon_target_pos_diff;

    causal::VectorCrossProductVariable act_horizon_ang_sin;
    causal::VectorDotProductVariable act_horizon_ang_cos;
    causal::ScalarReciprocalVariable act_horizon_ang_cos_recip;
    causal::ScalarProductVariable act_horizon_ang;

    causal::ScalarProductVariable needed_ang_vel;

    causal::ScalarFixedVariable double_scale_factor;
    causal::ScalarProxyVariable double_scale_factor_proxy;
    causal::ScalarProductVariable double_axel_dist;

    causal::ScalarProductVariable needed_ang_vel_double_axel_dist_prod;
    causal::ScalarProductVariable needed_steer;

    causal::ScalarMinVariable max_lim_steer;
    causal::ScalarMaxVariable actual_steer;

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
