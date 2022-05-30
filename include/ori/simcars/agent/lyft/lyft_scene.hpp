#pragma once

#include <ori/simcars/structures/stl/stl_dictionary.hpp>
#include <ori/simcars/agent/scene_abstract.hpp>
#include <ori/simcars/agent/lyft/lyft_declarations.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{
namespace lyft
{

class LyftScene : public AScene<LyftScene>
{
    mutable structures::stl::STLDictionary<uint32_t, std::shared_ptr<const IAgent>> ego_agent_dict;
    mutable structures::stl::STLDictionary<uint32_t, std::shared_ptr<const IAgent>> non_ego_agent_dict;

protected:
    void save_virt(std::ofstream& output_filestream) const override;
    void load_virt(std::ifstream& input_filestream) override;

public:
    temporal::Time get_earliest_birth() const override;
    temporal::Time get_last_non_simulated_death() const override;

    std::shared_ptr<const structures::stl::STLStackArray<std::shared_ptr<const IAgent>>> get_ego_agents() const override;
    std::shared_ptr<const structures::stl::STLStackArray<std::shared_ptr<const IAgent>>> get_non_ego_agents() const override;

    void perform_simulations(temporal::Time time) const override;
    std::shared_ptr<const IAgentStateHolder::State> perform_presimulation_checks(bool ego, uint32_t id, temporal::Time current_time, temporal::Duration time_step, std::shared_ptr<const IAgentStateHolder::State> state) const override;
    std::shared_ptr<const agent::IScene> fork_simulated_scene(bool ego, uint32_t id, temporal::Time fork_time, temporal::Duration time_step) const override;
};

}
}
}
}
