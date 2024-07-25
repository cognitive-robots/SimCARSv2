#pragma once

#include <ori/simcars/agents/plan_fwd_car.hpp>
#include <ori/simcars/agents/causal/variable_types/exogenous/fwd_car_action_fixed.hpp>
#include <ori/simcars/agents/causal/variable_types/endogenous/fwd_car_action_buffer.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class ActionInterventionFWDCar : public virtual PlanFWDCar
{
protected:
    void init_links() override;

    causal::FWDCarActionFixedVariable action_intervention;
    causal::FWDCarActionBufferVariable action_intervention_buff;

public:
    ActionInterventionFWDCar(FWDCarAction action);

    simcars::causal::IEndogenousVariable<FWDCarAction>* get_action_intervention_variable();
};

}
}
}
