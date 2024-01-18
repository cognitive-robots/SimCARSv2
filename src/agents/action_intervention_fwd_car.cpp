
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
    action.set_parent(&action_intervention);
}

ActionInterventionFWDCar::ActionInterventionFWDCar(FWDCarAction action) :
    action_intervention(action)
{
}

}
}
}
