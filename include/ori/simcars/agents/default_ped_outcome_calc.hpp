#pragma once

#include <ori/simcars/agents/ped_outcome_calc_interface.hpp>
#include <ori/simcars/agents/point_mass_env.hpp>
#include <ori/simcars/agents/goal_force_control_ped.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class DefaultPedOutcomeCalc : public virtual IPedOutcomeCalc
{
    GoalForceControlPed *control_ped;

public:
    DefaultPedOutcomeCalc(GoalForceControlPed *control_ped);

    PedOutcome calc_outcome(PedOutcomeCalcParameters const *parameters) const override;
    PedAction get_action() const override;
};

}
}
}
