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
    simcars::causal::ScalarFixedVariable mass;
    simcars::causal::ScalarProxyVariable mass_proxy;
    simcars::causal::ScalarReciprocalVariable mass_recip;

    simcars::causal::ScalarFixedVariable length;
    simcars::causal::ScalarProxyVariable length_proxy;
    simcars::causal::ScalarProductVariable length_squared;

    simcars::causal::ScalarFixedVariable width;
    simcars::causal::ScalarProxyVariable width_proxy;
    simcars::causal::ScalarProductVariable width_squared;

    simcars::causal::ScalarFixedVariable height;
    simcars::causal::ScalarProxyVariable height_proxy;

    simcars::causal::ScalarFixedVariable drag_area;
    simcars::causal::ScalarProxyVariable drag_area_proxy;

    simcars::causal::ScalarFixedVariable moi_scale_factor;
    simcars::causal::ScalarProductVariable moi_scaled_mass;

    simcars::causal::ScalarSumVariable span_squared;

    simcars::causal::ScalarProductVariable moi;
    simcars::causal::ScalarReciprocalVariable moi_recip;

    simcars::causal::VectorSocketVariable env_force;
    simcars::causal::VectorProxyVariable env_force_proxy;
    simcars::causal::VectorSocketVariable other_force;
    simcars::causal::VectorSumVariable total_force;

    simcars::causal::ScalarSocketVariable env_torque;
    simcars::causal::ScalarProxyVariable env_torque_proxy;
    simcars::causal::ScalarSocketVariable other_torque;
    simcars::causal::ScalarSumVariable total_torque;

    simcars::causal::VectorScalarProductVariable lin_acc;
    simcars::causal::VectorBufferVariable lin_acc_buff;
    simcars::causal::VectorPreviousTimeStepVariable prev_lin_acc;

    simcars::causal::ScalarProductVariable ang_acc;
    simcars::causal::ScalarBufferVariable ang_acc_buff;
    simcars::causal::ScalarPreviousTimeStepVariable prev_ang_acc;

    simcars::causal::VectorTimeStepSizeProductVariable lin_vel_diff;
    simcars::causal::VectorPreviousTimeStepVariable prev_lin_vel;
    simcars::causal::VectorSumVariable lin_vel;
    simcars::causal::VectorBufferVariable lin_vel_buff;

    simcars::causal::ScalarTimeStepSizeProductVariable ang_vel_diff;
    simcars::causal::ScalarPreviousTimeStepVariable prev_ang_vel;
    simcars::causal::ScalarSumVariable ang_vel;
    simcars::causal::ScalarBufferVariable ang_vel_buff;

    simcars::causal::VectorTimeStepSizeProductVariable pos_diff;
    simcars::causal::VectorPreviousTimeStepVariable prev_pos;
    simcars::causal::VectorSumVariable pos;
    simcars::causal::VectorBufferVariable pos_buff;

    simcars::causal::ScalarTimeStepSizeProductVariable rot_diff;
    simcars::causal::ScalarPreviousTimeStepVariable prev_rot;
    simcars::causal::ScalarSumVariable rot;
    simcars::causal::ScalarBufferVariable rot_buff;

    simcars::causal::ORectConstructionVariable rect;

public:
    RectRigidBody(FP_DATA_TYPE mass_value, FP_DATA_TYPE length_value, FP_DATA_TYPE width_value,
                  FP_DATA_TYPE height_value, FP_DATA_TYPE drag_area_value = 0.631);

    simcars::causal::IEndogenousVariable<FP_DATA_TYPE> const* get_mass_variable() const;
    simcars::causal::IEndogenousVariable<FP_DATA_TYPE> const* get_length_variable() const;
    simcars::causal::IEndogenousVariable<FP_DATA_TYPE> const* get_width_variable() const;
    simcars::causal::IEndogenousVariable<FP_DATA_TYPE> const* get_height_variable() const;
    simcars::causal::IEndogenousVariable<FP_DATA_TYPE> const* get_drag_area_variable() const;
    simcars::causal::IEndogenousVariable<geometry::Vec> const* get_env_force_variable() const;
    simcars::causal::IEndogenousVariable<geometry::Vec> const* get_lin_vel_variable() const;
    simcars::causal::IEndogenousVariable<geometry::Vec> const* get_pos_variable() const;
    simcars::causal::IEndogenousVariable<geometry::ORect> const* get_rect_variable() const;

    friend bool RectRigidBodyEnv::add_entity(RectRigidBody *rigid_body);
    friend bool RectRigidBodyEnv::remove_entity(RectRigidBody *rigid_body);
};

}
}
}
