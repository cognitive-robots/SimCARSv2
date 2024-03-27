
#include <ori/simcars/agents/fwd_car_action_intervention.hpp>

#include <ori/simcars/agents/fwd_car_action.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

void FWDCarActionIntervention::init_links()
{
    action.set_parent(&action_intervention_buff);
}

FWDCarActionIntervention::FWDCarActionIntervention(FWDCarAction action) :
    action_intervention(action),
    action_intervention_buff(&action_intervention)
{
}

simcars::causal::IEndogenousVariable<FWDCarAction>* FWDCarActionIntervention::get_action_intervention_variable()
{
    return &action_intervention_buff;
}

}
}
}
