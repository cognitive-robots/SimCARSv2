#pragma once

#include <ori/simcars/causal/variable_types/endogenous/id_proxy.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_proxy.hpp>
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

    causal::ScalarFixedVariable max_steer;
    causal::ScalarNegationVariable min_steer;

    causal::IdProxyVariable lane_val_goal_proxy;

    causal::TimeCurrentTimeDifferenceVariable time_error;
    causal::DurationSecondsCastVariable time_error_secs;
    causal::ScalarFixedVariable min_act_horizon_secs;
    causal::ScalarMaxVariable actual_act_horizon_secs;
    causal::ScalarReciprocalVariable actual_act_horizon_secs_recip;

    causal::VectorSocketVariable pos;
    causal::VectorProxyVariable pos_proxy;
    causal::VectorSocketVariable lin_vel;
    causal::VectorProxyVariable lin_vel_proxy;
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

    causal::ScalarSocketVariable axel_dist;
    causal::ScalarFixedVariable double_scale_factor;
    causal::ScalarProxyVariable double_scale_factor_proxy;
    causal::ScalarProductVariable double_axel_dist;

    causal::ScalarSocketVariable lon_lin_vel_recip;
    causal::ScalarProductVariable needed_ang_vel_double_axel_dist_prod;
    causal::ScalarProductVariable needed_steer;

    causal::ScalarMinVariable max_lim_steer;
    causal::ScalarMaxVariable actual_steer;

public:
    SteerControlFWDCar(map::IMap const *map, FP_DATA_TYPE max_abs_steer_value);

    void set_fwd_car(FWDCar *fwd_car);
};

}
}
}
