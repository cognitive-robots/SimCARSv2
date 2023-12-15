
#include <ori/simcars/agents/default_fwd_car_outcome_sim.hpp>

#include <ori/simcars/causal/variable_context.hpp>
#include <ori/simcars/agents/rect_rigid_body_sim.hpp>
#include <ori/simcars/agents/fwd_car_sim.hpp>
#include <ori/simcars/agents/full_control_fwd_car_sim.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

DefaultFWDCarOutcomeSim::DefaultFWDCarOutcomeSim(FullControlFWDCar const *control_fwd_car,
                                                 RectRigidBodyEnv const *rigid_body_env) :
    control_fwd_car(control_fwd_car), rigid_body_env(rigid_body_env)
{
}

FWDCarOutcome DefaultFWDCarOutcomeSim::sim_outcome(FWDCarAction const *action,
                                                   FWDCarSimParameters const *parameters) const
{
    temporal::Time start_time = simcars::causal::VariableContext::get_current_time();

    assert(action->speed_goal.time > start_time && action->lane_goal.time > start_time);

    RectRigidBody const *selected_rigid_body = control_fwd_car->get_fwd_car();

    RectRigidBodyEnv rigid_body_sim_env;

    structures::IArray<RectRigidBody const*> const *env_rigid_body_array =
            rigid_body_env->get_rigid_bodies();

    size_t i;
    for (i = 0; i < env_rigid_body_array->count(); ++i)
    {
        if ((*env_rigid_body_array)[i] == selected_rigid_body) continue;
        rigid_body_sim_env.add_rigid_body(
                    new RectRigidBodySim((*env_rigid_body_array)[i], start_time));
    }

    FWDCarSim *fwd_car_sim = new FWDCarSim(control_fwd_car->get_fwd_car(), start_time);
    FullControlFWDCarSim *control_fwd_car_sim = new FullControlFWDCarSim(control_fwd_car,
                                                                         start_time);
    control_fwd_car_sim->set_fwd_car(fwd_car_sim);
    rigid_body_sim_env.add_rigid_body(fwd_car_sim);


    temporal::Duration sim_horizon = std::chrono::duration_cast<temporal::Duration>(
                std::chrono::duration<FP_DATA_TYPE>(parameters->sim_horizon_secs));
    simcars::causal::VariableContext::set_current_time(start_time + sim_horizon);

    FWDCarOutcome outcome;

    outcome.lane_transitions =
            int8_t(control_fwd_car_sim->get_cumil_lane_trans_variable()->get_value());
    outcome.final_speed = fwd_car_sim->get_lon_lin_vel_variable()->get_value();
    outcome.max_env_force_mag = fwd_car_sim->get_max_env_force_mag_variable()->get_value();


    structures::IArray<RectRigidBody const*> const *sim_env_rigid_body_array =
            rigid_body_sim_env.get_rigid_bodies();

    for (i = 0; i < sim_env_rigid_body_array->count(); ++i)
    {
        delete (*sim_env_rigid_body_array)[i];
    }

    simcars::causal::VariableContext::set_current_time(start_time);

    return outcome;
}

}
}
}
