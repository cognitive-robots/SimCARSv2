
#include <ori/simcars/utils/exceptions.hpp>
#include <ori/simcars/agent/simulated_agent_abstract.hpp>

#include <cassert>

namespace ori
{
namespace simcars
{
namespace agent
{

ASimulatedAgent::ASimulatedAgent(std::shared_ptr<const IScene> scene, std::shared_ptr<const IAgent> base_agent, temporal::Time sim_start_time, temporal::Duration sim_time_step)
    : base_agent(base_agent), sim_start_time(sim_start_time), sim_time_step(sim_time_step), timestamp_to_state_dict((3 * sim_time_step) / 4, 10), scene(scene)
{
    assert (sim_start_time >= base_agent->get_birth() && sim_start_time <= base_agent->get_death());
}

ASimulatedAgent::ASimulatedAgent(std::shared_ptr<const IScene> scene, std::shared_ptr<const IAgent> base_agent, temporal::Duration sim_time_step) : ASimulatedAgent(scene, base_agent, base_agent->get_death(), sim_time_step) {}

temporal::Time ASimulatedAgent::get_sim_start_time() const
{
    return sim_start_time;
}

temporal::Duration ASimulatedAgent::get_sim_time_step() const
{
    return sim_time_step;
}

std::shared_ptr<const ASimulatedAgent::State> ASimulatedAgent::get_state(temporal::Time timestamp) const
{
    /*
     * Recursion would be really nice here, but in theory this would cause the amount we can "rewind" time to be restricted by
     * stack size limits. Not really a problem on a modern desktop, but potentially an issue for embedded devices with fewer
     * resources. Even on a desktop, if we wanted to simulate an hour of millisecond precision simulation - for whatever reason -
     * we would run in to trouble.
     */
    if (timestamp >= sim_start_time + sim_time_step)
    {
        size_t total_sim_step_count = (timestamp - sim_start_time).count() / sim_time_step.count();

        size_t current_sim_step_count = total_sim_step_count;
        while (current_sim_step_count > 0 && !timestamp_to_state_dict.contains(sim_start_time + current_sim_step_count * sim_time_step))
        {
            --current_sim_step_count;
        }

        while (current_sim_step_count < total_sim_step_count)
        {
            std::shared_ptr<const ASimulatedAgent::State> previous_state;
            if (current_sim_step_count > 0)
            {
                previous_state = timestamp_to_state_dict[sim_start_time + current_sim_step_count * sim_time_step];
            }
            else
            {
                previous_state = base_agent->get_state(sim_start_time);
            }
            previous_state = get_scene()->perform_presimulation_checks(
                        is_ego(),
                        get_id(),
                        sim_start_time + current_sim_step_count * sim_time_step,
                        sim_time_step,
                        previous_state);

            std::shared_ptr<ASimulatedAgent::State> current_state = simulate(previous_state, sim_time_step);
            ++current_sim_step_count;
            timestamp_to_state_dict.update(sim_start_time + current_sim_step_count * sim_time_step, current_state);
        }

        return timestamp_to_state_dict[sim_start_time + total_sim_step_count * sim_time_step];
    }
    else
    {
        return base_agent->get_state(timestamp);
    }
}

temporal::Time ASimulatedAgent::get_birth() const
{
    return base_agent->get_birth();
}

temporal::Time ASimulatedAgent::get_death() const
{
    return temporal::Time::max();
}

uint32_t ASimulatedAgent::get_id() const
{
    return base_agent->get_id();
}

bool ASimulatedAgent::is_ego() const
{
    return base_agent->is_ego();
}

bool ASimulatedAgent::is_ever_simulated() const
{
    return true;
}

bool ASimulatedAgent::is_simulated(temporal::Time timestamp) const
{
    return timestamp > sim_start_time;
}

FP_DATA_TYPE ASimulatedAgent::get_length() const
{
    return base_agent->get_length();
}

FP_DATA_TYPE ASimulatedAgent::get_width() const
{
    return base_agent->get_width();
}

ASimulatedAgent::Class ASimulatedAgent::get_class() const
{
    return base_agent->get_class();
}

std::shared_ptr<const IScene> ASimulatedAgent::get_scene() const
{
    return scene.lock();
}

}
}
}
