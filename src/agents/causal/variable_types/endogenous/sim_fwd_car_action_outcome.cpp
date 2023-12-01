
#include <ori/simcars/agents/causal/variable_types/endogenous/sim_fwd_car_action_outcome.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

SimFWDCarActionOutcome::SimFWDCarActionOutcome(
        simcars::causal::IEndogenousVariable<structures::stl::STLStackArray<FWDCarAction>> const *endogenous_parent,
        simcars::causal::IVariable<FWDCarSimParameters> const *other_parent,
        IFWDCarOutcomeSim const *fwd_car_outcome_sim) :
    ABinaryEndogenousVariable(endogenous_parent, other_parent),
    fwd_car_outcome_sim(fwd_car_outcome_sim) {}

structures::stl::STLStackArray<FWDCarOutcomeActionPair> SimFWDCarActionOutcome::get_value() const
{
    structures::stl::STLStackArray<FWDCarAction> actions = get_endogenous_parent()->get_value();
    FWDCarSimParameters sim_parameters = get_other_parent()->get_value();

    structures::stl::STLStackArray<FWDCarOutcomeActionPair> outcome_action_pairs;

    for (size_t i = 0; i < actions.count(); ++i)
    {
        outcome_action_pairs.push_back(
                    FWDCarOutcomeActionPair(fwd_car_outcome_sim->sim_outcome(&(actions[i]),
                                                                             &sim_parameters),
                                            actions[i]));
    }

    return outcome_action_pairs;
}

}
}
}
}
