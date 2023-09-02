#pragma once

#include <ori/simcars/causal/variable_types/endogenous/vector_angle_construction.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_xy_construction.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_y.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_norm.hpp>
#include <ori/simcars/causal/variable_types/endogenous/matrix_angle_construction.hpp>
#include <ori/simcars/causal/variable_types/endogenous/matrix_vector_product.hpp>
#include <ori/simcars/agents/rect_rigid_body.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class FWDCar : public RectRigidBody
{
protected:
    causal::ScalarFixedVariable wheel_radius;
    causal::ScalarReciprocalVariable wheel_radius_recip;

    causal::ScalarFixedVariable axel_dist;
    causal::ScalarNegationVariable neg_axel_dist;

    causal::ScalarFixedVariable cornering_stiffness;

    causal::VectorAngleConstructionVariable dir;

    causal::VectorDotProductVariable lon_lin_vel;
    causal::ScalarReciprocalVariable lon_lin_vel_recip;

    causal::VectorCrossProductVariable lat_lin_vel;

    causal::ScalarSocketVariable motor_torque;
    causal::ScalarProductVariable front_wheel_lon_force_mag;

    causal::ScalarFixedVariable rear_wheel_lon_force_mag;
    causal::ScalarProxyVariable rear_wheel_lon_force_mag_proxy;

    causal::ScalarProductVariable front_wheel_ang_lat_lin_vel;
    causal::ScalarSumVariable front_wheel_lat_lin_vel;
    causal::ScalarProductVariable neg_front_wheel_slip_ang_minus_steer;
    causal::ScalarNegationVariable front_wheel_slip_ang_minus_steer;
    causal::ScalarSocketVariable steer;
    causal::ScalarSumVariable front_wheel_slip_ang;
    causal::ScalarProductVariable front_wheel_lat_force_mag;

    causal::ScalarProductVariable rear_wheel_ang_lat_lin_vel;
    causal::ScalarSumVariable rear_wheel_lat_lin_vel;
    causal::ScalarProductVariable neg_rear_wheel_slip_ang;
    causal::ScalarNegationVariable rear_wheel_slip_ang;
    causal::ScalarProductVariable rear_wheel_lat_force_mag;

    causal::VectorXYConstructionVariable front_wheel_local_force;
    causal::MatrixAngleConstructionVariable steer_mat;
    causal::MatrixVectorProductVariable steered_front_wheel_local_force;

    causal::VectorXYConstructionVariable rear_wheel_local_force;

    causal::VectorYVariable front_wheel_torque_force_mag;
    causal::ScalarProductVariable front_wheel_torque;

    causal::VectorYVariable rear_wheel_torque_force_mag;
    causal::ScalarProductVariable rear_wheel_torque;

    causal::VectorSumVariable combined_wheel_local_force;
    causal::MatrixAngleConstructionVariable rot_mat;
    causal::MatrixVectorProductVariable combined_wheel_force;

    causal::ScalarSumVariable combined_wheel_torque;

    causal::VectorSocketVariable other_force;
    causal::ScalarSocketVariable other_torque;

    causal::VectorSumVariable total_wheel_force;
    causal::ScalarSumVariable total_wheel_torque;

public:
    FWDCar(FP_DATA_TYPE mass_value, FP_DATA_TYPE length_value, FP_DATA_TYPE width_value,
           FP_DATA_TYPE height_value, FP_DATA_TYPE wheel_radius_value,
           FP_DATA_TYPE axel_dist_value, FP_DATA_TYPE drag_area_value = 0.631,
           FP_DATA_TYPE cornering_stiffness_value = 49675.0);
};

}
}
}
