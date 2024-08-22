#pragma once

#include <ori/simcars/causal/variable_types/endogenous/scalar_absolute.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_min.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_max.hpp>
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

    simcars::causal::ScalarFixedVariable axle_dist;
    simcars::causal::ScalarProxyVariable axle_dist_proxy;
    simcars::causal::ScalarNegationVariable neg_axle_dist;

    simcars::causal::ScalarFixedVariable cornering_stiffness;

    simcars::causal::ScalarFixedVariable max_abs_slip_angle;
    simcars::causal::ScalarNegationVariable neg_max_abs_slip_angle;

    simcars::causal::VectorAngleConstructionVariable dir;

    simcars::causal::VectorDotProductVariable lon_lin_acc;
    simcars::causal::VectorDotProductVariable lon_lin_vel;
    simcars::causal::ScalarReciprocalVariable lon_lin_vel_recip;
    simcars::causal::ScalarAbsoluteVariable abs_lon_lin_vel;
    simcars::causal::ScalarReciprocalVariable abs_lon_lin_vel_recip;

    simcars::causal::VectorCrossProductVariable lat_lin_vel;

    simcars::causal::ScalarSocketVariable motor_torque;
    simcars::causal::ScalarBufferVariable motor_torque_buff;
    simcars::causal::ScalarProductVariable front_wheel_lon_force_mag;

    simcars::causal::ScalarFixedVariable rear_wheel_lon_force_mag;
    simcars::causal::ScalarProxyVariable rear_wheel_lon_force_mag_proxy;

    simcars::causal::ScalarProductVariable front_wheel_ang_lat_lin_vel;
    simcars::causal::ScalarSumVariable front_wheel_lat_lin_vel;
    simcars::causal::ScalarProductVariable neg_front_wheel_slip_ang_minus_steer;
    simcars::causal::ScalarNegationVariable front_wheel_slip_ang_minus_steer;
    simcars::causal::ScalarSocketVariable steer;
    simcars::causal::ScalarBufferVariable steer_buff;
    simcars::causal::ScalarSumVariable front_wheel_slip_ang;
    simcars::causal::ScalarMinVariable max_lim_front_wheel_slip_ang;
    simcars::causal::ScalarMaxVariable actual_front_wheel_slip_ang;
    simcars::causal::ScalarProductVariable front_wheel_lat_force_mag;

    simcars::causal::ScalarProductVariable rear_wheel_ang_lat_lin_vel;
    simcars::causal::ScalarSumVariable rear_wheel_lat_lin_vel;
    simcars::causal::ScalarProductVariable neg_rear_wheel_slip_ang;
    simcars::causal::ScalarNegationVariable rear_wheel_slip_ang;
    simcars::causal::ScalarMinVariable max_lim_rear_wheel_slip_ang;
    simcars::causal::ScalarMaxVariable actual_rear_wheel_slip_ang;
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

public:
    FWDCar(uint64_t id_value, FP_DATA_TYPE mass_value, FP_DATA_TYPE length_value,
           FP_DATA_TYPE width_value, FP_DATA_TYPE height_value, FP_DATA_TYPE wheel_radius_value,
           FP_DATA_TYPE axle_dist_value, FP_DATA_TYPE drag_area_value = 0.631,
           FP_DATA_TYPE cornering_stiffness_value = 49675.0,
           FP_DATA_TYPE max_abs_slip_angle_value = M_PI_2);
    FWDCar(FWDCar const &fwd_car);

    simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_wheel_radius_variable();
    simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_axle_dist_variable();
    simcars::causal::IEndogenousVariable<geometry::Vec>* get_dir_variable();
    simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_lon_lin_acc_variable();
    simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_lon_lin_vel_variable();
    simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_lon_lin_vel_recip_variable();
    simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_lat_lin_vel_variable();
    simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_front_wheel_ang_lat_lin_vel_variable();
    simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_front_wheel_slip_ang();
    simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_actual_front_wheel_slip_ang();
    simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_front_wheel_lat_force_mag_variable();
    simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_rear_wheel_ang_lat_lin_vel_variable();
    simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_rear_wheel_slip_ang();
    simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_actual_rear_wheel_slip_ang();
    simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_rear_wheel_lat_force_mag_variable();

    virtual simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_motor_torque_variable();
    virtual simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_steer_variable();

    friend void ControlFWDCar::set_fwd_car(FWDCar *fwd_car);
};

}
}
}
