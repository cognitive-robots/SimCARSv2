
#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/structures/stl/stl_read_only_concat_array.hpp>
#include <ori/simcars/agent/inattentive_simulated_agent.hpp>
#include <ori/simcars/agent/lyft/lyft_scene.hpp>
#include <ori/simcars/agent/lyft/lyft_agent.hpp>

#include <laudrup/lz4_stream/lz4_stream.hpp>
#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>

namespace ori
{
namespace simcars
{
namespace agent
{
namespace lyft
{

void LyftScene::save_virt(std::ofstream &output_filestream) const
{
    throw utils::NotImplementedException();
}

void LyftScene::load_virt(std::ifstream& input_filestream)
{
    lz4_stream::istream input_lz4_stream(input_filestream);
    rapidjson::BasicIStreamWrapper input_lz4_json_stream(input_lz4_stream);

    rapidjson::Document json_document;
    json_document.ParseStream(input_lz4_json_stream);

    for (const rapidjson::Value& json_document_element : json_document.GetArray())
    {
        std::shared_ptr<const IAgent> current_agent(new LyftAgent(shared_from_this(), json_document_element.GetObject()));
        if (current_agent->is_ego())
        {
            ego_agent_dict.update(current_agent->get_id(), current_agent);
        }
        else
        {
            non_ego_agent_dict.update(current_agent->get_id(), current_agent);
        }
    }
}

temporal::Time LyftScene::get_earliest_birth() const
{
    temporal::Time earliest_birth = temporal::Time::max();

    std::shared_ptr<const structures::IArray<std::shared_ptr<const IAgent>>> ego_agents = this->get_ego_agents();

    size_t i;
    for (i = 0; i < ego_agents->count(); ++i)
    {
        std::shared_ptr<const IAgent> agent = (*ego_agents)[i];
        earliest_birth = std::min(agent->get_birth(), earliest_birth);
    }

    std::shared_ptr<const structures::IArray<std::shared_ptr<const IAgent>>> non_ego_agents = this->get_non_ego_agents();

    for (i = 0; i < non_ego_agents->count(); ++i)
    {
        std::shared_ptr<const IAgent> agent = (*non_ego_agents)[i];
        earliest_birth = std::min(agent->get_birth(), earliest_birth);
    }

    return earliest_birth;
}

temporal::Time LyftScene::get_last_non_simulated_death() const
{
    temporal::Time latest_death = temporal::Time::min();

    std::shared_ptr<const structures::IArray<std::shared_ptr<const IAgent>>> ego_agents = this->get_ego_agents();

    size_t i;
    for (i = 0; i < ego_agents->count(); ++i)
    {
        std::shared_ptr<const IAgent> agent = (*ego_agents)[i];
        if (!agent->is_ever_simulated())
        {
            latest_death = std::max(agent->get_death(), latest_death);
        }
    }

    std::shared_ptr<const structures::IArray<std::shared_ptr<const IAgent>>> non_ego_agents = this->get_non_ego_agents();

    for (i = 0; i < non_ego_agents->count(); ++i)
    {
        std::shared_ptr<const IAgent> agent = (*non_ego_agents)[i];
        if (!agent->is_ever_simulated())
        {
            latest_death = std::max(agent->get_death(), latest_death);
        }
    }

    return latest_death;
}

std::shared_ptr<const structures::stl::STLStackArray<std::shared_ptr<const IAgent>>> LyftScene::get_ego_agents() const
{
    std::shared_ptr<const structures::IArray<std::shared_ptr<const IAgent>>> ego_agents = ego_agent_dict.get_values();

    std::shared_ptr<structures::stl::STLStackArray<std::shared_ptr<const IAgent>>> new_ego_agents(
                new structures::stl::STLStackArray<std::shared_ptr<const IAgent>>());

    size_t i;
    for (i = 0; i < ego_agents->count(); ++i)
    {
        std::shared_ptr<const IAgent> agent = (*ego_agents)[i];
        new_ego_agents->push_back(agent);
    }

    return new_ego_agents;
}

std::shared_ptr<const structures::stl::STLStackArray<std::shared_ptr<const IAgent>>> LyftScene::get_non_ego_agents() const
{
    std::shared_ptr<const structures::IArray<std::shared_ptr<const IAgent>>> non_ego_agents = non_ego_agent_dict.get_values();

    std::shared_ptr<structures::stl::STLStackArray<std::shared_ptr<const IAgent>>> new_non_ego_agents(
                new structures::stl::STLStackArray<std::shared_ptr<const IAgent>>());

    size_t i;
    for (i = 0; i < non_ego_agents->count(); ++i)
    {
        std::shared_ptr<const IAgent> agent = (*non_ego_agents)[i];
        new_non_ego_agents->push_back(agent);
    }

    return new_non_ego_agents;
}

void LyftScene::perform_simulations(temporal::Time time) const
{
    std::shared_ptr<const structures::IArray<std::shared_ptr<const IAgent>>> ego_agents = this->get_ego_agents();
    std::shared_ptr<const structures::IArray<std::shared_ptr<const IAgent>>> non_ego_agents = this->get_non_ego_agents();

    size_t i;
    for (i = 0; i < ego_agents->count(); ++i)
    {
        std::shared_ptr<const IAgent> agent = (*ego_agents)[i];
        try
        {
            agent->get_state(time);
        }
        catch (const std::out_of_range&) {}
    }
    for (i = 0; i < non_ego_agents->count(); ++i)
    {
        std::shared_ptr<const IAgent> agent = (*non_ego_agents)[i];
        try
        {
            agent->get_state(time);
        }
        catch (const std::out_of_range&) {}
    }
}

std::shared_ptr<const IAgentStateHolder::State> LyftScene::perform_presimulation_checks(bool ego, uint32_t id, temporal::Time current_time, temporal::Duration time_step, std::shared_ptr<const IAgentStateHolder::State> state) const
{
    std::shared_ptr<const IAgent> agent;
    if (ego)
    {
        agent = ego_agent_dict[id];
    }
    else
    {
        agent = non_ego_agent_dict[id];
    }

    std::shared_ptr<const structures::IArray<std::shared_ptr<const IAgent>>> ego_agents = this->get_ego_agents();
    std::shared_ptr<const structures::IArray<std::shared_ptr<const IAgent>>> non_ego_agents = this->get_non_ego_agents();

    bool collision = false;

    size_t i;
    for (i = 0; i < ego_agents->count(); ++i)
    {
        std::shared_ptr<const IAgent> other_agent = (*ego_agents)[i];

        if (ego && id == other_agent->get_id())
        {
            continue;
        }

        try
        {
            std::shared_ptr<const geometry::ORect> other_agent_collision_box = other_agent->get_collision_box(current_time);
            if (agent->get_collision_box(current_time)->check_collision(*other_agent_collision_box))
            {
                if (!other_agent->is_simulated(current_time))
                {
                    std::shared_ptr<IAgent> new_simulated_agent(new InattentiveSimulatedAgent(shared_from_this(), other_agent, current_time - time_step, time_step));
                    ego_agent_dict.update(other_agent->get_id(), new_simulated_agent);
                    new_simulated_agent->get_state(current_time);
                }

                collision = true;
            }
        }
        catch (const std::out_of_range&) {}
    }

    for (i = 0; i < non_ego_agents->count(); ++i)
    {
        std::shared_ptr<const IAgent> other_agent = (*non_ego_agents)[i];

        if (!ego && id == other_agent->get_id())
        {
            continue;
        }

        try
        {
            std::shared_ptr<const geometry::ORect> agent_collision_box = agent->get_collision_box(current_time);
            std::shared_ptr<const geometry::ORect> other_agent_collision_box = other_agent->get_collision_box(current_time);
            if (agent_collision_box->check_collision(*other_agent_collision_box))
            {
                if (!other_agent->is_simulated(current_time))
                {
                    std::shared_ptr<IAgent> new_simulated_agent(new InattentiveSimulatedAgent(shared_from_this(), other_agent, current_time - time_step, time_step));
                    non_ego_agent_dict.update(other_agent->get_id(), new_simulated_agent);
                    new_simulated_agent->get_state(current_time);
                }

                collision = true;
            }
        }
        catch (const std::out_of_range&) {}
    }

    if (collision)
    {
        std::shared_ptr<IAgentStateHolder::State> new_state(new IAgentStateHolder::State(*state));
        new_state->linear_velocity = geometry::Vec::Zero();
        new_state->linear_acceleration = geometry::Vec::Zero();
        new_state->angular_velocity = 0.0f;
        new_state->angular_acceleration = 0.0f;
        new_state->status = IAgentStateHolder::Status::FATAL;
        return new_state;
    }
    else
    {
        return state;
    }
}

std::shared_ptr<const agent::IScene> LyftScene::fork_simulated_scene(bool ego, uint32_t id, temporal::Time fork_time, temporal::Duration time_step) const
{
    std::shared_ptr<agent::lyft::LyftScene> forked_scene(new agent::lyft::LyftScene());

    forked_scene->ego_agent_dict = this->ego_agent_dict;
    forked_scene->non_ego_agent_dict = this->non_ego_agent_dict;

    std::shared_ptr<const IAgent> agent_to_fork;
    if (ego)
    {
        agent_to_fork = forked_scene->ego_agent_dict[id];
    }
    else
    {
        agent_to_fork = forked_scene->non_ego_agent_dict[id];
    }

    std::shared_ptr<const IAgent> forked_agent(new InattentiveSimulatedAgent(forked_scene, agent_to_fork, fork_time, time_step));

    if (ego)
    {
        forked_scene->ego_agent_dict.update(id, forked_agent);
    }
    else
    {
        forked_scene->non_ego_agent_dict.update(id, forked_agent);
    }

    return forked_scene;
}

}
}
}
}
