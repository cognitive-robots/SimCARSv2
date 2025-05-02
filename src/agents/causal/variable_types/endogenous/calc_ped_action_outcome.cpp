
#include <ori/simcars/agents/causal/variable_types/endogenous/calc_ped_action_outcome.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{
namespace causal
{

CalcPedActionOutcomeVariable::CalcPedActionOutcomeVariable(
        simcars::causal::IVariable<PedSimParameters> *parent,
        IPedOutcomeCalc const *ped_outcome_calc) :
    AUnaryEndogenousVariable(parent),
    ped_outcome_calc(ped_outcome_calc) {}

bool CalcPedActionOutcomeVariable::get_value(PedOutcomeActionPair &val) const
{
    PedSimParameters sim_parameters;
    if (get_parent()->get_value(sim_parameters))
    {
        PedOutcomeCalcParameters outcome_calc_parameters = sim_parameters;

        val = PedOutcomeActionPair(ped_outcome_calc->calc_outcome(&outcome_calc_parameters),
                                   ped_outcome_calc->get_action());

        return true;
    }
    else
    {
        return false;
    }
}

bool CalcPedActionOutcomeVariable::set_value(PedOutcomeActionPair const &val)
{
    PedSimParameters sim_parameters;
    if (get_parent()->get_value(sim_parameters))
    {
        PedOutcomeCalcParameters outcome_calc_parameters = sim_parameters;

        PedOutcomeActionPair outcome_action_pair(
                    ped_outcome_calc->calc_outcome(&outcome_calc_parameters),
                    ped_outcome_calc->get_action());

        return val == outcome_action_pair;
    }
    else
    {
        return true;
    }
}

}
}
}
}
