#pragma once

#include <ori/simcars/structures/set_interface.hpp>
#include <ori/simcars/structures/stl/stl_dictionary.hpp>
#include <ori/simcars/agent/declarations.hpp>
#include <ori/simcars/agent/driving_simulation_scene_interface.hpp>
#include <ori/simcars/agent/driving_simulator_interface.hpp>
#include <ori/simcars/agent/driving_scene_abstract.hpp>
#include <ori/simcars/agent/driving_simulation_agent.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class DrivingSimulationScene : public virtual ADrivingScene, public virtual IDrivingSimulationScene
{
    geometry::Vec min_spatial_limits, max_spatial_limits;
    temporal::Time min_temporal_limit, max_temporal_limit;
    temporal::Duration time_step;

    mutable temporal::Time furthest_simulation_time;

    IDrivingSimulator const *simulator;

    structures::stl::STLDictionary<std::string, IDrivingAgent*> simulated_driving_agent_dict;
    structures::stl::STLDictionary<std::string, IDrivingAgent*> non_simulated_driving_agent_dict;

public:
    ~DrivingSimulationScene();

    static DrivingSimulationScene* construct_from(IDrivingScene *driving_scene,
                                                        IDrivingSimulator const *driving_simulator,
                                                        temporal::Duration time_step,
                                                        temporal::Time simulation_start_time,
                                                        structures::ISet<std::string> *starting_agent_names = nullptr);
    static DrivingSimulationScene* construct_from(IDrivingScene *driving_scene,
                                                        IDrivingSimulator const *driving_simulator,
                                                        temporal::Duration simulation_time_step,
                                                        temporal::Time simulation_start_time,
                                                        temporal::Time simulation_end_time,
                                                        structures::ISet<std::string> *starting_agent_names = nullptr);

    geometry::Vec get_min_spatial_limits() const override;
    geometry::Vec get_max_spatial_limits() const override;

    temporal::Duration get_time_step() const override;

    temporal::Time get_min_temporal_limit() const override;
    temporal::Time get_max_temporal_limit() const override;

    structures::IArray<IDrivingAgent const*>* get_driving_agents() const override;
    IDrivingAgent const* get_driving_agent(std::string const &driving_agent_name) const override;

    void simulate(temporal::Time time) override;

    structures::IArray<IDrivingAgent*>* get_mutable_driving_agents() override;
    IDrivingAgent* get_mutable_driving_agent(std::string const &driving_agent_name) override;
};

}
}
}
