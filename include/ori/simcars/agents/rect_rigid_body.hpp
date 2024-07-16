#pragma once

#include <ori/simcars/causal/variable_types/exogenous/id_fixed.hpp>
#include <ori/simcars/causal/variable_types/exogenous/scalar_fixed.hpp>
#include <ori/simcars/causal/variable_types/exogenous/scalar_socket.hpp>
#include <ori/simcars/causal/variable_types/exogenous/vector_socket.hpp>
#include <ori/simcars/causal/variable_types/endogenous/id_proxy.hpp>
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
#include <ori/simcars/agents/point_mass.hpp>
#include <ori/simcars/agents/rect_rigid_body_env.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class RectRigidBody : public virtual PointMass
{
protected:
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

    simcars::causal::ScalarSocketVariable dist_headway;
    simcars::causal::ScalarBufferVariable dist_headway_buff;

    simcars::causal::ScalarSocketVariable env_torque;
    simcars::causal::ScalarBufferVariable env_torque_buff;
    simcars::causal::ScalarSocketVariable other_torque;
    simcars::causal::ScalarBufferVariable other_torque_buff;
    simcars::causal::ScalarSumVariable total_torque;

    simcars::causal::ScalarProductVariable ang_acc;
    simcars::causal::ScalarBufferVariable ang_acc_buff;
    simcars::causal::ScalarPreviousTimeStepVariable prev_ang_acc;

    simcars::causal::ScalarTimeStepSizeProductVariable ang_vel_diff;
    simcars::causal::ScalarPreviousTimeStepVariable prev_ang_vel;
    simcars::causal::ScalarSumVariable ang_vel;
    simcars::causal::ScalarBufferVariable ang_vel_buff;

    simcars::causal::ScalarTimeStepSizeProductVariable rot_diff;
    simcars::causal::ScalarPreviousTimeStepVariable prev_rot;
    simcars::causal::ScalarSumVariable rot;
    simcars::causal::ScalarBufferVariable rot_buff;

    simcars::causal::ORectConstructionVariable rect;

public:
    RectRigidBody(uint64_t id_value, FP_DATA_TYPE mass_value, FP_DATA_TYPE length_value,
                  FP_DATA_TYPE width_value, FP_DATA_TYPE height_value,
                  FP_DATA_TYPE drag_area_value = 0.631);
    RectRigidBody(RectRigidBody const &rect_rigid_body);

    temporal::Time get_min_time() const override;
    temporal::Time get_max_time() const override;

    simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_length_variable();
    simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_width_variable();
    simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_height_variable();
    simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_drag_area_variable();
    simcars::causal::IEndogenousVariable<geometry::ORect>* get_rect_variable();

    virtual simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_dist_headway_variable();
    virtual simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_env_torque_variable();
    virtual simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_other_torque_variable();
    virtual simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_ang_acc_variable();
    virtual simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_ang_vel_variable();
    virtual simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_rot_variable();

    friend bool RectRigidBodyEnv::add_rigid_body(RectRigidBody *rigid_body);
    friend bool RectRigidBodyEnv::remove_rigid_body(RectRigidBody *rigid_body);
};

}
}
}
