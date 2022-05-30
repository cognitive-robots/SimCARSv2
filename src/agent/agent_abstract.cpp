
#include <ori/simcars/agent/agent_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

std::shared_ptr<const geometry::ORect> AAgent::get_collision_box(temporal::Time timestamp) const
{
    std::shared_ptr<const AAgent::State> state = get_state(timestamp);
    std::shared_ptr<const geometry::ORect> collision_box(
                new geometry::ORect(state->position, this->get_length(), this->get_width(), state->rotation));
    return collision_box;
}

std::shared_ptr<const structures::IArray<std::shared_ptr<const IAgent>>> AAgent::get_other_agents() const
{
    std::shared_ptr<const IScene> scene = get_scene();

    std::shared_ptr<const structures::IArray<std::shared_ptr<const IAgent>>> ego_agents =
            scene->get_ego_agents();
    std::shared_ptr<const structures::IArray<std::shared_ptr<const IAgent>>> non_ego_agents =
            scene->get_non_ego_agents();
    std::shared_ptr<structures::IArray<std::shared_ptr<const IAgent>>> other_agents(
            new structures::stl::STLStackArray<std::shared_ptr<const IAgent>>(
                    ego_agents->count() + non_ego_agents->count() - 1));

    size_t i, j;
    if (is_ego())
    {
        for (i = 0, j = 0; i < ego_agents->count(); ++i, ++j)
        {
            if (*((*ego_agents)[i]) == *this)
            {
                --j;
            }
            else
            {
                (*other_agents)[j] = (*ego_agents)[i];
            }
        }

        for (i = 0, j = ego_agents->count() - 1; i < non_ego_agents->count(); ++i, ++j)
        {
            (*other_agents)[j] = (*non_ego_agents)[i];
        }
    }
    else
    {
        for (i = 0, j = 0; i < ego_agents->count(); ++i, ++j)
        {
            (*other_agents)[j] = (*ego_agents)[i];
        }

        for (i = 0, j = ego_agents->count(); i < non_ego_agents->count(); ++i, ++j)
        {
            if (*((*non_ego_agents)[i]) == *this)
            {
                --j;
            }
            else
            {
                (*other_agents)[j] = (*non_ego_agents)[i];
            }
        }
    }

    return other_agents;
}

bool AAgent::operator ==(const IAgent& agent) const
{
    return this->is_ego() == agent.is_ego() && this->get_id() == agent.get_id();
}

}
}
}
