#pragma once

#include <ori/simcars/agents/plan_fwd_car.hpp>
#include <ori/simcars/agents/causal/variable_types/exogenous/fwd_car_action_fixed.hpp>

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

public:
    ActionInterventionFWDCar(FWDCarAction action);
};

}
}
}
