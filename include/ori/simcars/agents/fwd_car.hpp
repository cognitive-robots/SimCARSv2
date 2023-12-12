#pragma once

#include <ori/simcars/causal/variable_types/endogenous/vector_angle_construction.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_xy_construction.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_y.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_norm.hpp>
#include <ori/simcars/causal/variable_types/endogenous/matrix_angle_construction.hpp>
#include <ori/simcars/causal/variable_types/endogenous/matrix_vector_product.hpp>
#include <ori/simcars/agents/declarations.hpp>
#include <ori/simcars/agents/rect_rigid_body.hpp>
#include <ori/simcars/agents/control_fwd_car.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class FWDCar : public virtual RectRigidBody
{
protected:
    simcars::causal::ScalarFixedVariable wheel_radius;
    simcars::causal::ScalarProxyVariable wheel_radius_proxy;
    simcars::causal::ScalarReciprocalVariable wheel_radius_recip;

    simcars::causal::ScalarFixedVariable axel_dist;
    simcars::causal::ScalarProxyVariable axel_dist_proxy;
    simcars::causal::ScalarNegationVariable neg_axel_dist;

    simcars::causal::ScalarFixedVariable cornering_stiffness;

    simcars::causal::VectorAngleConstructionVariable dir;

    simcars::causal::VectorDotProductVariable lon_lin_vel;
    simcars::causal::ScalarReciprocalVariable lon_lin_vel_recip;

    simcars::causal::VectorCrossProductVariable lat_lin_vel;

    simcars::causal::ScalarSocketVariable motor_torque;
    simcars::causal::ScalarProductVariable front_wheel_lon_force_mag;

    simcars::causal::ScalarFixedVariable rear_wheel_lon_force_mag;
    simcars::causal::ScalarProxyVariable rear_wheel_lon_force_mag_proxy;

    simcars::causal::ScalarProductVariable front_wheel_ang_lat_lin_vel;
    simcars::causal::ScalarSumVariable front_wheel_lat_lin_vel;
    simcars::causal::ScalarProductVariable neg_front_wheel_slip_ang_minus_steer;
    simcars::causal::ScalarNegationVariable front_wheel_slip_ang_minus_steer;
    simcars::causal::ScalarSocketVariable steer;
    simcars::causal::ScalarSumVariable front_wheel_slip_ang;
    simcars::causal::ScalarProductVariable front_wheel_lat_force_mag;

    simcars::causal::ScalarProductVariable rear_wheel_ang_lat_lin_vel;
    simcars::causal::ScalarSumVariable rear_wheel_lat_lin_vel;
    simcars::causal::ScalarProductVariable neg_rear_wheel_slip_ang;
    simcars::causal::ScalarNegationVariable rear_wheel_slip_ang;
    simcars::causal::ScalarProductVariable rear_wheel_lat_force_mag;

    simcars::causal::VectorXYConstructionVariable front_wheel_local_force;
    simcars::causal::MatrixAngleConstructionVariable steer_mat;
    simcars::causal::MatrixVectorProductVariable steered_front_wheel_local_force;

    simcars::causal::VectorXYConstructionVariable rear_wheel_local_force;

    simcars::causal::VectorYVariable front_wheel_torque_force_mag;
    simcars::causal::ScalarProductVariable front_wheel_torque;

    simcars::causal::VectorYVariable rear_wheel_torque_force_mag;
    simcars::causal::ScalarProductVariable rear_wheel_torque;

    simcars::causal::VectorSumVariable combined_wheel_local_force;
    simcars::causal::MatrixAngleConstructionVariable rot_mat;
    simcars::causal::MatrixVectorProductVariable combined_wheel_force;

    simcars::causal::ScalarSumVariable combined_wheel_torque;

    simcars::causal::VectorSocketVariable other_force;
    simcars::causal::ScalarSocketVariable other_torque;

    simcars::causal::VectorSumVariable total_wheel_force;
    simcars::causal::ScalarSumVariable total_wheel_torque;

public:
    FWDCar(FP_DATA_TYPE mass_value, FP_DATA_TYPE length_value, FP_DATA_TYPE width_value,
           FP_DATA_TYPE height_value, FP_DATA_TYPE wheel_radius_value,
           FP_DATA_TYPE axel_dist_value, FP_DATA_TYPE drag_area_value = 0.631,
           FP_DATA_TYPE cornering_stiffness_value = 49675.0);

    simcars::causal::IEndogenousVariable<FP_DATA_TYPE> const* get_wheel_radius_variable() const;
    simcars::causal::IEndogenousVariable<FP_DATA_TYPE> const* get_axel_dist_variable() const;
    simcars::causal::IEndogenousVariable<FP_DATA_TYPE> const* get_lon_lin_vel_variable() const;
    simcars::causal::IEndogenousVariable<FP_DATA_TYPE> const* get_lon_lin_vel_recip_variable() const;
    simcars::causal::IEndogenousVariable<geometry::Vec> const* get_dir_variable() const;

    friend void ControlFWDCar::set_fwd_car(FWDCar *fwd_car);
};

}
}
}
