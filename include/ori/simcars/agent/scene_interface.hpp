#pragma once

#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/temporal/typedefs.hpp>
#include <ori/simcars/agent/declarations.hpp>
#include <ori/simcars/agent/agent_interface.hpp>

#include <string>
#include <memory>

namespace ori
{
namespace simcars
{
namespace agent
{

class IScene
{
public:
    virtual ~IScene() = default;

    virtual temporal::Time get_earliest_birth() const = 0;
    virtual temporal::Time get_last_non_simulated_death() const = 0;

    virtual std::shared_ptr<const structures::stl::STLStackArray<std::shared_ptr<const IAgent>>> get_ego_agents() const = 0;
    virtual std::shared_ptr<const structures::stl::STLStackArray<std::shared_ptr<const IAgent>>> get_non_ego_agents() const = 0;

    virtual void perform_simulations(temporal::Time time) const = 0;
    virtual std::shared_ptr<const IAgentStateHolder::State> perform_presimulation_checks(bool ego, uint32_t id, temporal::Time current_time, temporal::Duration time_step, std::shared_ptr<const IAgentStateHolder::State> state) const = 0;
    virtual std::shared_ptr<const IScene> fork_simulated_scene(bool ego, uint32_t id, temporal::Time fork_time, temporal::Duration time_step) const = 0;

    virtual void save(const std::string& output_file_path_str) const = 0;
};

}
}
}
