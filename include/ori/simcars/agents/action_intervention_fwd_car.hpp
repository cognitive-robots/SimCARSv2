#pragma once

#include <ori/simcars/causal/variable_types/exogenous/id_fixed.hpp>
#include <ori/simcars/causal/variable_types/exogenous/scalar_fixed.hpp>
#include <ori/simcars/causal/variable_types/exogenous/time_fixed.hpp>
#include <ori/simcars/agents/plan_fwd_car.hpp>

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

    causal::ScalarFixedVariable action_speed_goal_val;
    causal::TimeFixedVariable action_speed_goal_time;

    causal::IdFixedVariable action_lane_goal_val;
    causal::TimeFixedVariable action_lane_goal_time;

public:
    ActionInterventionFWDCar(FWDCarAction action);
};

}
}
}
