#pragma once

#include <ori/simcars/causal/variable_types/exogenous/time_fixed.hpp>
#include <ori/simcars/causal/variable_types/endogenous/scalar_time_conditional.hpp>
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

    simcars::causal::VectorTimeConditionalVariable sim_lin_acc;
    simcars::causal::VectorTimeConditionalVariable sim_lin_vel;
    simcars::causal::VectorTimeConditionalVariable sim_pos;

    simcars::causal::ScalarTimeConditionalVariable sim_ang_acc;
    simcars::causal::ScalarTimeConditionalVariable sim_ang_vel;
    simcars::causal::ScalarTimeConditionalVariable sim_rot;

public:
    RectRigidBodySim(RectRigidBody const *rect_rigid_body, temporal::Time start_time);
};

}
}
}
