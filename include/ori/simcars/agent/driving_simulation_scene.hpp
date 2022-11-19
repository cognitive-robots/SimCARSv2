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
    mutable IDrivingSceneState const *furthest_simulation_state;

    IDrivingSimulator const *simulator;

    structures::stl::STLDictionary<std::string, IDrivingAgent const*> driving_agent_dict;

public:
    ~DrivingSimulationScene();

    static DrivingSimulationScene const* construct_from(IDrivingScene const *driving_scene,
                                                        IDrivingSimulator const *driving_simulator,
                                                        temporal::Duration time_step,
                                                        temporal::Time simulation_start_time);
    static DrivingSimulationScene const* construct_from(IDrivingScene const *driving_scene,
                                                        IDrivingSimulator const *driving_simulator,
                                                        temporal::Duration simulation_time_step,
                                                        temporal::Time simulation_start_time,
                                                        temporal::Time simulation_end_time);

    geometry::Vec get_min_spatial_limits() const override;
    geometry::Vec get_max_spatial_limits() const override;

    temporal::Time get_min_temporal_limit() const override;
    temporal::Time get_max_temporal_limit() const override;

    structures::IArray<IEntity const*>* get_entities() const override;
    IEntity const* get_entity(std::string const &entity_name) const override;

    structures::IArray<IDrivingAgent const*>* get_driving_agents() const override;
    IDrivingAgent const* get_driving_agent(std::string const &driving_agent_name) const override;

    temporal::Duration get_time_step() const override
    {
        return time_step;
    }

    void simulate_and_propogate(temporal::Time time) const override;
};

}
}
}
