
#include <ori/simcars/utils/exceptions.hpp>
#include <ori/simcars/structures/stl/stl_stack_array.hpp>
#include <ori/simcars/structures/stl/stl_concat_array.hpp>
#include <ori/simcars/agent/view_driving_scene_state.hpp>
#include <ori/simcars/agent/driving_simulation_scene.hpp>

#include <iostream>

namespace ori
{
namespace simcars
{
namespace agent
{

DrivingSimulationScene::~DrivingSimulationScene()
{
    structures::IArray<IDrivingAgent*> const *simulated_driving_agents =
            simulated_driving_agent_dict.get_values();

    for (size_t i = 0; i < simulated_driving_agents->count(); ++i)
    {
        delete (*simulated_driving_agents)[i];
    }
}

DrivingSimulationScene* DrivingSimulationScene::construct_from(
        IDrivingScene *driving_scene,
        IDrivingSimulator const *driving_simulator,
        temporal::Duration simulation_time_step,
        temporal::Time simulation_start_time,
        structures::ISet<std::string> *starting_agent_names)
{
    return DrivingSimulationScene::construct_from(
                driving_scene,
                driving_simulator,
                simulation_time_step,
                simulation_start_time,
                driving_scene->get_max_temporal_limit(),
                starting_agent_names);
}

DrivingSimulationScene* DrivingSimulationScene::construct_from(
        IDrivingScene *driving_scene,
        IDrivingSimulator const *driving_simulator,
        temporal::Duration simulation_time_step,
        temporal::Time simulation_start_time,
        temporal::Time simulation_end_time,
        structures::ISet<std::string> *starting_agent_names)
{
    if (simulation_start_time < driving_scene->get_min_temporal_limit())
    {
        throw std::invalid_argument("Simulation start time is before earliest event for original scene");
    }

    if (simulation_start_time > driving_scene->get_max_temporal_limit())
    {
        throw std::invalid_argument("Simulation start time is after latest event for original scene");
    }

    if (simulation_start_time > simulation_end_time)
    {
        throw std::invalid_argument("Simulation start time is after simulation end time");
    }

    DrivingSimulationScene *new_driving_scene = new DrivingSimulationScene();

    new_driving_scene->min_spatial_limits = driving_scene->get_min_spatial_limits();
    new_driving_scene->max_spatial_limits = driving_scene->get_max_spatial_limits();
    new_driving_scene->min_temporal_limit = driving_scene->get_min_temporal_limit();
    new_driving_scene->max_temporal_limit = simulation_end_time;
    new_driving_scene->furthest_simulation_time = simulation_start_time;
    new_driving_scene->time_step = simulation_time_step;
    new_driving_scene->simulator = driving_simulator;

    structures::IArray<IDrivingAgent*> *driving_agents =
            driving_scene->get_mutable_driving_agents();

    for (size_t i = 0; i < driving_agents->count(); ++i)
    {
        bool starting_agent = starting_agent_names == nullptr ||
                starting_agent_names->contains((*driving_agents)[i]->get_name());

        try
        {
            DrivingSimulationAgent *driving_simulation_agent =
                        new DrivingSimulationAgent(
                            (*driving_agents)[i],
                            new_driving_scene,
                            simulation_start_time,
                            simulation_end_time,
                            starting_agent);

            new_driving_scene->simulated_driving_agent_dict.update(
                        driving_simulation_agent->get_name(), driving_simulation_agent);
        }
        catch (std::invalid_argument)
        {
            new_driving_scene->non_simulated_driving_agent_dict.update(
                        (*driving_agents)[i]->get_name(), (*driving_agents)[i]);
        }
    }

    delete driving_agents;

    return new_driving_scene;
}

// Not updated by simulation
geometry::Vec DrivingSimulationScene::get_min_spatial_limits() const
{
    return min_spatial_limits;
}

// Not updated by simulation
geometry::Vec DrivingSimulationScene::get_max_spatial_limits() const
{
    return max_spatial_limits;
}

temporal::Duration DrivingSimulationScene::get_time_step() const
{
    return time_step;
}

temporal::Time DrivingSimulationScene::get_min_temporal_limit() const
{
    return min_temporal_limit;
}

temporal::Time DrivingSimulationScene::get_max_temporal_limit() const
{
    return max_temporal_limit;
}

structures::IArray<IDrivingAgent const*>* DrivingSimulationScene::get_driving_agents() const
{
    structures::stl::STLConcatArray<IDrivingAgent const*> *driving_agents =
            new structures::stl::STLConcatArray<IDrivingAgent const*>(2);

    structures::IArray<IDrivingAgent const*> *non_simulated_driving_agents =
            new structures::stl::STLStackArray<IDrivingAgent const*>(non_simulated_driving_agent_dict.count());
    cast_array(*non_simulated_driving_agent_dict.get_values(), *non_simulated_driving_agents);
    driving_agents->get_array(0) = non_simulated_driving_agents;

    structures::IArray<IDrivingAgent const*> *simulated_driving_agents =
            new structures::stl::STLStackArray<IDrivingAgent const*>(simulated_driving_agent_dict.count());
    cast_array(*(simulated_driving_agent_dict.get_values()), *simulated_driving_agents);
    driving_agents->get_array(1) = simulated_driving_agents;

    return driving_agents;
}

IDrivingAgent const* DrivingSimulationScene::get_driving_agent(std::string const &driving_agent_name) const
{
    if (non_simulated_driving_agent_dict.contains(driving_agent_name))
    {
        return non_simulated_driving_agent_dict[driving_agent_name];
    }
    else
    {
        return simulated_driving_agent_dict[driving_agent_name];
    }
}

IDrivingSimulationScene* DrivingSimulationScene::driving_simulation_scene_deep_copy() const
{
    DrivingSimulationScene *new_driving_simulation_scene = new DrivingSimulationScene();

    new_driving_simulation_scene->min_spatial_limits = this->min_spatial_limits;
    new_driving_simulation_scene->max_spatial_limits = this->max_spatial_limits;
    new_driving_simulation_scene->time_step = this->time_step;
    new_driving_simulation_scene->min_temporal_limit = this->min_temporal_limit;
    new_driving_simulation_scene->max_temporal_limit = this->max_temporal_limit;

    new_driving_simulation_scene->furthest_simulation_time = this->furthest_simulation_time;

    // Might as well reuse the same simulator, it doesn't actually contain any data
    new_driving_simulation_scene->simulator = this->simulator;

    structures::IArray<IDrivingAgent*> const *simulated_driving_agents =
            this->simulated_driving_agent_dict.get_values();

    size_t i;

    for (i = 0; i < simulated_driving_agents->count(); ++i)
    {
        new_driving_simulation_scene->simulated_driving_agent_dict.update(
                    (*simulated_driving_agents)[i]->get_name(),
                    (*simulated_driving_agents)[i]->driving_agent_deep_copy());
    }

    structures::IArray<IDrivingAgent*> const *non_simulated_driving_agents =
            this->non_simulated_driving_agent_dict.get_values();

    for (i = 0; i < non_simulated_driving_agents->count(); ++i)
    {
        new_driving_simulation_scene->non_simulated_driving_agent_dict.update(
                    (*non_simulated_driving_agents)[i]->get_name(),
                    (*non_simulated_driving_agents)[i]->driving_agent_deep_copy());
    }

    return new_driving_simulation_scene;
}

void DrivingSimulationScene::simulate(temporal::Time time)
{
    ViewDrivingSceneState furthest_simulation_state(this, furthest_simulation_time);
    ViewDrivingSceneState new_state(this, furthest_simulation_time + time_step);
    while (furthest_simulation_time < std::min(time, max_temporal_limit))
    {
        simulator->simulate_driving_scene(&furthest_simulation_state, &new_state, time_step);

        furthest_simulation_time += time_step;

        furthest_simulation_state.set_time(furthest_simulation_time);
        new_state.set_time(furthest_simulation_time + time_step);
    }
}

structures::IArray<IDrivingAgent*>* DrivingSimulationScene::get_mutable_driving_agents()
{
    structures::stl::STLConcatArray<IDrivingAgent*> *driving_agents =
            new structures::stl::STLConcatArray<IDrivingAgent*>(2);

    driving_agents->get_array(0) =
            new structures::stl::STLStackArray<IDrivingAgent*>(
                non_simulated_driving_agent_dict.get_values());

    driving_agents->get_array(1) =
            new structures::stl::STLStackArray<IDrivingAgent*>(
                simulated_driving_agent_dict.get_values());

    return driving_agents;
}

IDrivingAgent* DrivingSimulationScene::get_mutable_driving_agent(std::string const &driving_agent_name)
{
    if (non_simulated_driving_agent_dict.contains(driving_agent_name))
    {
        return non_simulated_driving_agent_dict[driving_agent_name];
    }
    else
    {
        return simulated_driving_agent_dict[driving_agent_name];
    }
}

}
}
}
