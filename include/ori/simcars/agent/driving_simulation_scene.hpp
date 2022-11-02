#pragma once

#include <ori/simcars/structures/stl/stl_dictionary.hpp>
#include <ori/simcars/agent/declarations.hpp>
#include <ori/simcars/agent/simulation_scene_interface.hpp>
#include <ori/simcars/agent/driving_simulator_interface.hpp>
#include <ori/simcars/agent/driving_scene_abstract.hpp>
#include <ori/simcars/agent/driving_simulation_agent.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class DrivingSimulationScene : public virtual ADrivingScene, public virtual ISimulationScene
{
    geometry::Vec min_spatial_limits, max_spatial_limits;
    temporal::Time min_temporal_limit, max_temporal_limit;
    temporal::Duration time_step;

    mutable temporal::Time furthest_simulation_time;
    mutable std::shared_ptr<const IDrivingSceneState> furthest_simulation_state;

    std::shared_ptr<const IDrivingSimulator> simulator;

    structures::stl::STLDictionary<std::string, std::shared_ptr<const IDrivingAgent>> driving_agent_dict;

public:
    static std::shared_ptr<const DrivingSimulationScene> construct_from(std::shared_ptr<const IDrivingScene> driving_scene,
                                                                        std::shared_ptr<const IDrivingSimulator> driving_simulator,
                                                                        temporal::Duration time_step,
                                                                        temporal::Time simulation_start_time);
    static std::shared_ptr<const DrivingSimulationScene> construct_from(std::shared_ptr<const IDrivingScene> driving_scene,
                                                                        std::shared_ptr<const IDrivingSimulator> driving_simulator,
                                                                        temporal::Duration simulation_time_step,
                                                                        temporal::Time simulation_start_time,
                                                                        temporal::Time simulation_end_time);

    geometry::Vec get_min_spatial_limits() const override;
    geometry::Vec get_max_spatial_limits() const override;

    temporal::Time get_min_temporal_limit() const override;
    temporal::Time get_max_temporal_limit() const override;

    std::shared_ptr<structures::IArray<std::shared_ptr<const IEntity>>> get_entities() const override;
    std::shared_ptr<const IEntity> get_entity(const std::string& entity_name) const override;

    std::shared_ptr<structures::IArray<std::shared_ptr<const IDrivingAgent>>> get_driving_agents() const override;
    std::shared_ptr<const IDrivingAgent> get_driving_agent(const std::string& driving_agent_name) const override;

    temporal::Duration get_time_step() const override
    {
        return time_step;
    }

    void simulate_and_propogate(temporal::Time time) const override;
};

}
}
}
