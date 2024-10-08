
#include <ori/simcars/agents/default_fwd_car_outcome_sim.hpp>

#include <ori/simcars/causal/variable_context.hpp>
#include <ori/simcars/agents/rect_rigid_body_sim.hpp>
#include <ori/simcars/agents/fwd_car_sim.hpp>
#include <ori/simcars/agents/full_control_fwd_car_sim.hpp>
#include <ori/simcars/agents/action_intervention_fwd_car.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

DefaultFWDCarOutcomeSim::DefaultFWDCarOutcomeSim(FullControlFWDCar *control_fwd_car,
                                                 RectRigidBodyEnv *rigid_body_env) :
    control_fwd_car(control_fwd_car), rigid_body_env(rigid_body_env)
{
}

FWDCarOutcome DefaultFWDCarOutcomeSim::sim_outcome(FWDCarAction const *action,
                                                   FWDCarSimParameters const *parameters) const
{
    temporal::Time start_time = simcars::causal::VariableContext::get_current_time();
    temporal::Duration time_step_size = simcars::causal::VariableContext::get_time_step_size();

    // WARNING: Temporarily removed assertion, might not be necessary
    //assert(action->speed_goal.time > start_time && action->lane_goal.time > start_time);

    FWDCar *fwd_car = control_fwd_car->get_fwd_car();

    /*
    RectRigidBodyEnv rigid_body_sim_env;

    structures::IArray<RectRigidBody*> const *env_rigid_body_array =
            rigid_body_env->get_rigid_bodies();

    // TODO: Change this to just use the actual agents rather than copying, since this will fail to
    // keep simulated agents as such
    size_t i;
    for (i = 0; i < env_rigid_body_array->count(); ++i)
    {
        if ((*env_rigid_body_array)[i] == selected_rigid_body) continue;
        // Simulating all agents is intractable, so just simulate the agent we are planning for
        //rigid_body_sim_env.add_rigid_body(
        //            new RectRigidBodySim((*env_rigid_body_array)[i], start_time));
        rigid_body_sim_env.add_rigid_body(new RectRigidBody(*(*env_rigid_body_array)[i]));
    }
    */

    FWDCarSim *fwd_car_sim = new FWDCarSim(fwd_car, start_time);
    FullControlFWDCarSim control_fwd_car_sim(control_fwd_car, start_time - time_step_size);
    ActionInterventionFWDCar plan_fwd_car_intervention(*action);
    control_fwd_car_sim.set_fwd_car(fwd_car_sim);
    plan_fwd_car_intervention.set_control_fwd_car(&control_fwd_car_sim);

    //rigid_body_sim_env.add_rigid_body(fwd_car_sim);
    rigid_body_env->remove_rigid_body(fwd_car);
    rigid_body_env->add_rigid_body(fwd_car_sim);


    temporal::Duration sim_horizon = std::chrono::duration_cast<temporal::Duration>(
                std::chrono::duration<FP_DATA_TYPE>(parameters->sim_horizon_secs));
    simcars::causal::VariableContext::set_current_time(start_time + sim_horizon);

    // TODO: Either utilise the FWD Car Outcome construction variable or remove it from the codebase
    FWDCarOutcome outcome;

    bool res = control_fwd_car_sim.get_cumil_lane_trans_variable()->get_value(outcome.lane_transitions);
    if (!res)
    {
        throw std::runtime_error("Could not get cumilative lane transitions");
    }
    res = fwd_car_sim->get_lon_lin_vel_variable()->get_value(outcome.final_speed);
    if (!res)
    {
        throw std::runtime_error("Could not get final speed");
    }
    res = fwd_car_sim->get_dist_headway_variable()->get_value(outcome.dist_headway);
    if (!res)
    {
        throw std::runtime_error("Could not get distance headway");
    }
    res = fwd_car_sim->get_max_env_force_mag_variable()->get_value(outcome.max_env_force_mag);
    if (!res)
    {
        throw std::runtime_error("Could not get maximum environment force magnitude");
    }
    structures::stl::STLStackArray<uint64_t> lane_encaps;
    res = control_fwd_car_sim.get_lane_encaps_variable()->get_value(lane_encaps);
    if (!res)
    {
        throw std::runtime_error("Could not get encapsulating lanes");
    }
    outcome.action_done = (std::abs(outcome.final_speed - action->speed_goal.val) <=
            parameters->action_done_final_speed_threshold) &&
            lane_encaps.contains(action->lane_goal.val);


    rigid_body_env->remove_rigid_body(fwd_car_sim);
    rigid_body_env->add_rigid_body(fwd_car);

    delete fwd_car_sim;

    /*
    structures::IArray<RectRigidBody*> const *sim_env_rigid_body_array =
            rigid_body_sim_env.get_rigid_bodies();

    for (i = 0; i < sim_env_rigid_body_array->count(); ++i)
    {
        delete (*sim_env_rigid_body_array)[i];
    }
    */

    simcars::causal::VariableContext::set_current_time(start_time);

    return outcome;
}

}
}
}
