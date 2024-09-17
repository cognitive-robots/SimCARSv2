#pragma once

#include <ori/simcars/causal/variable_types/exogenous/time_fixed.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_max.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_time_conditional.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_norm.hpp>
#include <ori/simcars/causal/variable_types/endogenous/vector_time_conditional.hpp>
#include <ori/simcars/agents/point_mass_sim.hpp>
#include <ori/simcars/agents/rect_rigid_body.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class RectRigidBodySim : public virtual RectRigidBody, public virtual PointMassSim
{
    RectRigidBody const* const original_rect_rigid_body;

protected:
    simcars::causal::ScalarTimeConditionalVariable sim_dist_headway;

    simcars::causal::ScalarTimeConditionalVariable sim_env_torque;
    simcars::causal::ScalarTimeConditionalVariable sim_other_torque;
    simcars::causal::ScalarTimeConditionalVariable sim_ang_acc;
    simcars::causal::ScalarTimeConditionalVariable sim_ang_vel;
    simcars::causal::ScalarTimeConditionalVariable sim_rot;

public:
    RectRigidBodySim(RectRigidBody *rect_rigid_body, temporal::Time start_time);

    temporal::Time get_min_time() const override;
    temporal::Time get_max_time() const override;

    simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_dist_headway_variable() override;
    simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_env_torque_variable() override;
    simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_other_torque_variable() override;
    simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_ang_acc_variable() override;
    simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_ang_vel_variable() override;
    simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* get_rot_variable() override;
};

}
}
}
