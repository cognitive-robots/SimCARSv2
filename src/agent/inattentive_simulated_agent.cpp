
#include <ori/simcars/agent/inattentive_simulated_agent.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

InattentiveSimulatedAgent::InattentiveSimulatedAgent(std::shared_ptr<const IScene> scene, std::shared_ptr<const IAgent> base_agent,
                                                     temporal::Time sim_start_time, temporal::Duration sim_time_step)
    : ASimulatedAgent(scene, base_agent, sim_start_time, sim_time_step), trig_buff(geometry::TrigBuff::get_instance()) {}

std::shared_ptr<InattentiveSimulatedAgent::State> InattentiveSimulatedAgent::simulate(
        std::shared_ptr<const ASimulatedAgent::State> previous_state,
        temporal::Duration time_diff) const
{
    return simulate_physics(previous_state, time_diff);
}

std::shared_ptr<InattentiveSimulatedAgent::State> InattentiveSimulatedAgent::simulate_physics(
        std::shared_ptr<const ASimulatedAgent::State> previous_state,
        temporal::Duration time_diff) const
{
    std::shared_ptr<ASimulatedAgent::State> state(new ASimulatedAgent::State(*previous_state));

    // Angular acceleration will not change
    state->angular_velocity += state->angular_acceleration * time_diff.count();
    state->rotation += 0.5f * (previous_state->angular_velocity + state->angular_velocity) * time_diff.count();

    state->linear_acceleration = trig_buff->get_rot_mat(state->rotation - previous_state->rotation) * state->linear_acceleration;
    state->linear_velocity += 0.5f * (previous_state->linear_acceleration + state->linear_acceleration) * time_diff.count();
    state->position += 0.5f * (previous_state->linear_velocity + state->linear_velocity) * time_diff.count();

    return state;
}

}
}
}
