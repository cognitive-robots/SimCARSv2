
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
    speed_val_goal.set_parent(&action_speed_goal_val);
    speed_time_goal.set_parent(&action_speed_goal_time);
    lane_val_goal.set_parent(&action_lane_goal_val);
    lane_time_goal.set_parent(&action_lane_goal_time);
}

ActionInterventionFWDCar::ActionInterventionFWDCar(FWDCarAction action) :
    action_speed_goal_val(action.speed_goal.val),
    action_speed_goal_time(action.speed_goal.time),
    action_lane_goal_val(action.lane_goal.val),
    action_lane_goal_time(action.lane_goal.time)
{
}

}
}
}
