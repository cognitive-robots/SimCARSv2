
#include <ori/simcars/agents/action_intervention_fwd_car.hpp>

#include <ori/simcars/agents/fwd_car_action.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

void ActionInterventionFWDCar::init_links()
{
    action.set_parent(&action_intervention_buff);
}

ActionInterventionFWDCar::ActionInterventionFWDCar(FWDCarAction action) :
    action_intervention(action),
    action_intervention_buff(&action_intervention)
{
}

simcars::causal::IEndogenousVariable<FWDCarAction>* ActionInterventionFWDCar::get_action_intervention_variable()
{
    return &action_intervention_buff;
}

}
}
}
