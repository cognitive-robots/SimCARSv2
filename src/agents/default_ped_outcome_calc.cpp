
#include <ori/simcars/agents/default_ped_outcome_calc.hpp>

#include <ori/simcars/causal/variable_context.hpp>
#include <ori/simcars/agents/ped.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

DefaultPedOutcomeCalc::DefaultPedOutcomeCalc(GoalForceControlPed *control_ped) :
    control_ped(control_ped)
{
}

PedOutcome DefaultPedOutcomeCalc::calc_outcome(PedOutcomeCalcParameters const *parameters) const
{
    Ped *ped = control_ped->get_ped();

    // TODO: Either utilise the Ped Outcome construction variable or remove it from the codebase
    PedOutcome outcome;
    bool res = ped->get_pos_variable()->get_value(outcome.pos);
    if (!res)
    {
        throw std::runtime_error("Could not get position");
    }
    res = ped->get_min_neighbour_dist_variable()->get_value(outcome.min_neighbour_dist);
    if (!res)
    {
        throw std::runtime_error("Could not get minimum neighbour distance");
    }
    // Temporarily removed while carrying out experiments, since it is not easy to calculate this
    // for pre-simulation time steps, and we are not doing reward profile estimation within this
    // domain
    //geometry::Vec pos_diff;
    //res = control_ped->get_pos_diff_variable()->get_value(pos_diff);
    //if (!res)
    //{
    //    throw std::runtime_error("Could not get position difference");
    //}
    //outcome.action_done = pos_diff.norm() <= parameters->action_done_node_dist_threshold;

    return outcome;
}

PedAction DefaultPedOutcomeCalc::get_action() const
{
    PedAction action;
    // Temporarily commented out until experiments are done
    //control_ped->get_action_variable()->get_value(action);
    return action;
}

}
}
}
