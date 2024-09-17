#pragma once

#include <ori/simcars/agents/ped_outcome_sim_interface.hpp>
#include <ori/simcars/agents/point_mass_env.hpp>
#include <ori/simcars/agents/goal_force_control_ped.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class DefaultPedOutcomeSim : public virtual IPedOutcomeSim
{
    GoalForceControlPed *control_ped;
    PointMassEnv *point_mass_env;

public:
    DefaultPedOutcomeSim(GoalForceControlPed *control_ped, PointMassEnv *point_mass_env);

    PedOutcome sim_outcome(PedAction const *action,
                           PedSimParameters const *parameters) const override;
};

}
}
}
