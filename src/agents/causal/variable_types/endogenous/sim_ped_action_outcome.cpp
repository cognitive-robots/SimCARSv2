
#include <ori/simcars/agents/causal/variable_types/endogenous/sim_ped_action_outcome.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

SimPedActionOutcomeVariable::SimPedActionOutcomeVariable(
        simcars::causal::IEndogenousVariable<structures::stl::STLStackArray<PedAction>> *endogenous_parent,
        simcars::causal::IVariable<PedSimParameters> *other_parent,
        IPedOutcomeSim const *ped_outcome_sim) :
    ABinaryEndogenousVariable(endogenous_parent, other_parent),
    ped_outcome_sim(ped_outcome_sim) {}

bool SimPedActionOutcomeVariable::get_value(PedOutcomeActionPairs &val) const
{
    structures::stl::STLStackArray<PedAction> actions;
    PedSimParameters sim_parameters;
    if (get_endogenous_parent()->get_value(actions) && get_other_parent()->get_value(sim_parameters))
    {
        val.clear();

        for (size_t i = 0; i < actions.count(); ++i)
        {
            val.push_back(PedOutcomeActionPair(ped_outcome_sim->sim_outcome(
                                                   &(actions[i]), &sim_parameters), actions[i]));
        }

        return true;
    }
    else
    {
        return false;
    }
}

bool SimPedActionOutcomeVariable::set_value(PedOutcomeActionPairs const &val)
{
    structures::stl::STLStackArray<PedAction> actions;
    PedSimParameters sim_parameters;
    if (get_endogenous_parent()->get_value(actions) && get_other_parent()->get_value(sim_parameters))
    {
        for (size_t i = 0; i < actions.count(); ++i)
        {
            PedOutcomeActionPair outcome_action_pair(
                        ped_outcome_sim->sim_outcome(&(actions[i]), &sim_parameters), actions[i]);
            if (outcome_action_pair != val[i])
            {
                return false;
            }
        }
    }
    return true;
}

}
}
}
}
