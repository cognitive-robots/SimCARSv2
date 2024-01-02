
#include <ori/simcars/agents/causal/variable_types/endogenous/sim_fwd_car_action_outcome.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

SimFWDCarActionOutcomeVariable::SimFWDCarActionOutcomeVariable(
        simcars::causal::IEndogenousVariable<structures::stl::STLStackArray<FWDCarAction>> *endogenous_parent,
        simcars::causal::IVariable<FWDCarSimParameters> *other_parent,
        IFWDCarOutcomeSim const *fwd_car_outcome_sim) :
    ABinaryEndogenousVariable(endogenous_parent, other_parent),
    fwd_car_outcome_sim(fwd_car_outcome_sim) {}

bool SimFWDCarActionOutcomeVariable::get_value(structures::stl::STLStackArray<FWDCarOutcomeActionPair> &val) const
{
    structures::stl::STLStackArray<FWDCarAction> actions;
    FWDCarSimParameters sim_parameters;
    if (get_endogenous_parent()->get_value(actions) && get_other_parent()->get_value(sim_parameters))
    {
        val.clear();

        for (size_t i = 0; i < actions.count(); ++i)
        {
            val.push_back(FWDCarOutcomeActionPair(fwd_car_outcome_sim->sim_outcome(&(actions[i]),
                                                                                 &sim_parameters),
                                                  actions[i]));
        }

        return true;
    }
    else
    {
        return false;
    }
}

bool SimFWDCarActionOutcomeVariable::set_value(structures::stl::STLStackArray<FWDCarOutcomeActionPair> const &val)
{
    structures::stl::STLStackArray<FWDCarAction> actions;
    FWDCarSimParameters sim_parameters;
    if (get_endogenous_parent()->get_value(actions) && get_other_parent()->get_value(sim_parameters))
    {
        for (size_t i = 0; i < actions.count(); ++i)
        {
            FWDCarOutcomeActionPair outcome_action_pair(
                        fwd_car_outcome_sim->sim_outcome(&(actions[i]), &sim_parameters),
                        actions[i]);
            if (outcome_action_pair != val[i])
            {
                return false;
            }
        }

        return true;
    }
    else
    {
        return false;
    }
}

}
}
}
}
