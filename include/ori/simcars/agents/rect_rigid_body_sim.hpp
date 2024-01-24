#pragma once

#include <ori/simcars/causal/variable_types/exogenous/time_fixed.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_max.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_time_conditional.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_norm.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_time_conditional.hpp>
#include <ori/simcars/agents/rect_rigid_body.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class RectRigidBodySim : public virtual RectRigidBody
{
protected:
    simcars::causal::TimeFixedVariable sim_start_time;

    simcars::causal::VectorTimeConditionalVariable sim_env_force;
    simcars::causal::VectorTimeConditionalVariable sim_other_force;
    simcars::causal::VectorTimeConditionalVariable sim_lin_acc;
    simcars::causal::VectorTimeConditionalVariable sim_lin_vel;
    simcars::causal::VectorTimeConditionalVariable sim_pos;

    simcars::causal::ScalarTimeConditionalVariable sim_env_torque;
    simcars::causal::ScalarTimeConditionalVariable sim_other_torque;
    simcars::causal::ScalarTimeConditionalVariable sim_ang_acc;
    simcars::causal::ScalarTimeConditionalVariable sim_ang_vel;
    simcars::causal::ScalarTimeConditionalVariable sim_rot;

    simcars::causal::ScalarFixedVariable zero_max_env_force_mag;
    simcars::causal::ScalarProxyVariable zero_max_env_force_mag_proxy;

    simcars::causal::VectorNormVariable env_force_mag;

    simcars::causal::ScalarTimeConditionalVariable sim_max_env_force_mag;
    simcars::causal::ScalarPreviousTimeStepVariable prev_max_env_force_mag;

    simcars::causal::ScalarMaxVariable max_env_force_mag;

public:
    RectRigidBodySim(RectRigidBody *rect_rigid_body, temporal::Time start_time);

    simcars::causal::IEndogenousVariable<geometry::Vec>* get_env_force_variable() override;
    simcars::causal::IEndogenousVariable<geometry::Vec>* get_other_force_variable() override;
    simcars::causal::IEndogenousVariable<geometry::Vec>* get_lin_acc_variable() override;
    simcars::causal::IEndogenousVariable<geometry::Vec>* get_lin_vel_variable() override;
    simcars::causal::IEndogenousVariable<geometry::Vec>* get_pos_variable() override;
    simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_env_torque_variable() override;
    simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_other_torque_variable() override;
    simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_ang_acc_variable() override;
    simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_ang_vel_variable() override;
    simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_rot_variable() override;

    simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_max_env_force_mag_variable();
};

}
}
}
