#pragma once

#include <ori/simcars/causal/variable_types/exogenous/scalar_fixed.hpp>
#include <ori/simcars/causal/variable_types/exogenous/scalar_socket.hpp>
#include <ori/simcars/causal/variable_types/exogenous/vector_socket.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_proxy.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_reciprocal.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_sum.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_binary_mean.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_product.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_time_step_size_product.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_previous_time_step.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_buffer.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_proxy.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_sum.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_binary_mean.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_scalar_product.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_time_step_size_product.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_previous_time_step.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_buffer.hpp>
#include <ori/simcars/causal/variable_types/endogenous/o_rect_construction.hpp>
#include <ori/simcars/agents/declarations.hpp>
#include <ori/simcars/agents/rect_rigid_body_env.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class RectRigidBody
{
protected:
    causal::ScalarFixedVariable mass;
    causal::ScalarProxyVariable mass_proxy;
    causal::ScalarReciprocalVariable mass_recip;

    causal::ScalarFixedVariable length;
    causal::ScalarProxyVariable length_proxy;
    causal::ScalarProductVariable length_squared;

    causal::ScalarFixedVariable width;
    causal::ScalarProxyVariable width_proxy;
    causal::ScalarProductVariable width_squared;

    causal::ScalarFixedVariable height;
    causal::ScalarProxyVariable height_proxy;

    causal::ScalarFixedVariable drag_area;
    causal::ScalarProxyVariable drag_area_proxy;

    causal::ScalarFixedVariable moi_scale_factor;
    causal::ScalarProductVariable moi_scaled_mass;

    causal::ScalarSumVariable span_squared;

    causal::ScalarProductVariable moi;
    causal::ScalarReciprocalVariable moi_recip;

    causal::VectorSocketVariable env_force;
    causal::VectorProxyVariable env_force_proxy;
    causal::VectorSocketVariable other_force;
    causal::VectorSumVariable total_force;

    causal::ScalarSocketVariable env_torque;
    causal::ScalarProxyVariable env_torque_proxy;
    causal::ScalarSocketVariable other_torque;
    causal::ScalarSumVariable total_torque;

    causal::VectorScalarProductVariable lin_acc;
    causal::VectorBufferVariable lin_acc_buff;
    causal::VectorPreviousTimeStepVariable prev_lin_acc;

    causal::ScalarProductVariable ang_acc;
    causal::ScalarBufferVariable ang_acc_buff;
    causal::ScalarPreviousTimeStepVariable prev_ang_acc;

    causal::VectorTimeStepSizeProductVariable lin_vel_diff;
    causal::VectorPreviousTimeStepVariable prev_lin_vel;
    causal::VectorSumVariable lin_vel;
    causal::VectorBufferVariable lin_vel_buff;
    causal::VectorBinaryMeanVariable lin_vel_mean;

    causal::ScalarTimeStepSizeProductVariable ang_vel_diff;
    causal::ScalarPreviousTimeStepVariable prev_ang_vel;
    causal::ScalarSumVariable ang_vel;
    causal::ScalarBufferVariable ang_vel_buff;
    causal::ScalarBinaryMeanVariable ang_vel_mean;

    causal::VectorTimeStepSizeProductVariable pos_diff;
    causal::VectorPreviousTimeStepVariable prev_pos;
    causal::VectorSumVariable pos;
    causal::VectorBufferVariable pos_buff;

    causal::ScalarTimeStepSizeProductVariable rot_diff;
    causal::ScalarPreviousTimeStepVariable prev_rot;
    causal::ScalarSumVariable rot;
    causal::ScalarBufferVariable rot_buff;

    causal::ORectConstructionVariable rect;

public:
    RectRigidBody(FP_DATA_TYPE mass_value, FP_DATA_TYPE length_value, FP_DATA_TYPE width_value,
                  FP_DATA_TYPE height_value, FP_DATA_TYPE drag_area_value = 0.631);

    causal::IEndogenousVariable<FP_DATA_TYPE> const* get_mass_variable() const;
    causal::IEndogenousVariable<FP_DATA_TYPE> const* get_length_variable() const;
    causal::IEndogenousVariable<FP_DATA_TYPE> const* get_width_variable() const;
    causal::IEndogenousVariable<FP_DATA_TYPE> const* get_height_variable() const;
    causal::IEndogenousVariable<FP_DATA_TYPE> const* get_drag_area_variable() const;
    causal::IEndogenousVariable<geometry::Vec> const* get_lin_vel_variable() const;
    causal::IEndogenousVariable<geometry::Vec> const* get_pos_variable() const;
    causal::IEndogenousVariable<geometry::ORect> const* get_rect_variable() const;

    friend bool RectRigidBodyEnv::add_entity(RectRigidBody *rigid_body);
    friend bool RectRigidBodyEnv::remove_entity(RectRigidBody *rigid_body);
};

}
}
}
