
#include <ori/simcars/agents/action_intervention_ped.hpp>

#include <ori/simcars/agents/ped_action.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

void ActionInterventionPed::init_links()
{
    action.set_parent(&action_intervention_buff);
}

ActionInterventionPed::ActionInterventionPed(PedAction action) : action_intervention(action),
    action_intervention_buff(&action_intervention)
{
}

simcars::causal::IEndogenousVariable<PedAction>* ActionInterventionPed::get_action_intervention_variable()
{
    return &action_intervention_buff;
}

}
}
}
