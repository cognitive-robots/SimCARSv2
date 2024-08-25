#pragma once

#include <ori/simcars/agents/plan_ped.hpp>
#include <ori/simcars/agents/causal/variable_types/exogenous/ped_action_fixed.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/ped_action_buffer.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class ActionInterventionPed : public virtual PlanPed
{
protected:
    void init_links() override;

    causal::PedActionFixedVariable action_intervention;
    causal::PedActionBufferVariable action_intervention_buff;

public:
    ActionInterventionPed(PedAction action);

    simcars::causal::IEndogenousVariable<PedAction>* get_action_intervention_variable();
};

}
}
}
