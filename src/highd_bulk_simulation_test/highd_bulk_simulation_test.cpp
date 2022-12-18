
#include <ori/simcars/structures/stl/stl_set.hpp>
#include <ori/simcars/geometry/trig_buff.hpp>
#include <ori/simcars/map/highd/highd_map.hpp>
#include <ori/simcars/agent/driving_goal_extraction_scene.hpp>
#include <ori/simcars/agent/basic_fp_action_sampler.hpp>
#include <ori/simcars/agent/basic_driving_agent_controller.hpp>
#include <ori/simcars/agent/basic_driving_simulator.hpp>
#include <ori/simcars/agent/driving_simulation_scene.hpp>
#include <ori/simcars/agent/highd/highd_scene.hpp>

#include <iostream>
#include <exception>
#include <random>

#define NUMBER_OF_AGENTS 100
#define NUMBER_OF_SIMULATED_AGENTS 2
#define NUMBER_OF_SCENES 100

using namespace ori::simcars;
using namespace std::chrono;

void simulate(agent::IDrivingScene const *simulated_scene)
{
    structures::IArray<agent::IDrivingAgent const*> *driving_agents =
            simulated_scene->get_driving_agents();

    for (size_t i = 0; i < driving_agents->count(); ++i)
    {
        agent::IDrivingAgent const *driving_agent = (*driving_agents)[i];
        agent::IVariable<geometry::Vec> const *position_variable =
                driving_agent->get_position_variable();
        geometry::Vec position;
        bool result = position_variable->get_value(driving_agent->get_last_event_time(), position);
        if (!result)
        {
            std::cout << "Failed to get position variable value for agent " << i << " at end of scene" << std::endl;
        }
    }

    delete driving_agents;
}

int main(int argc, char *argv[])
{
    if (argc < 4)
    {
        std::cerr << "Usage: ./highd_simcars_demo recording_meta_file_path tracks_meta_file_path tracks_file_path" << std::endl;
        return -1;
    }


    structures::ISet<std::string> *agent_names = new structures::stl::STLSet<std::string>;

    size_t i, j;
    for (i = 1; i <= NUMBER_OF_AGENTS; ++i)
    {
        agent_names->insert("non_ego_vehicle_" + std::to_string(i));
    }


    geometry::TrigBuff::init_instance(360000, geometry::AngleType::RADIANS);

    time_point<high_resolution_clock> start_time;
    microseconds time_elapsed;

    std::cout << "Beginning map load" << std::endl;

    start_time = high_resolution_clock::now();

    map::IMap<uint8_t> const *map;

    try
    {
        map = map::highd::HighDMap::load(argv[1]);
    }
    catch (std::exception const &e)
    {
        std::cerr << "Exception occured during map load:" << std::endl << e.what() << std::endl;
        return -1;
    }

    time_elapsed = duration_cast<microseconds>(high_resolution_clock::now() - start_time);

    std::cout << "Finished map load (" << time_elapsed.count() << " μs)" << std::endl;

    std::cout << "Beginning scene load" << std::endl;

    start_time = high_resolution_clock::now();

    agent::IDrivingScene *scene;

    try
    {
        scene = agent::highd::HighDScene::load(argv[2], argv[3], agent_names);
    }
    catch (std::exception const &e)
    {
        std::cerr << "Exception occured during scene load:" << std::endl << e.what() << std::endl;
        return -1;
    }

    time_elapsed = duration_cast<microseconds>(high_resolution_clock::now() - start_time);

    std::cout << "Finished scene load (" << time_elapsed.count() << " μs)" << std::endl;

    delete agent_names;

    std::cout << "Beginning action extraction" << std::endl;

    start_time = high_resolution_clock::now();

    agent::IDrivingScene *scene_with_actions;

    try
    {
        scene_with_actions = agent::DrivingGoalExtractionScene<uint8_t>::construct_from(scene, map);
    }
    catch (std::exception const &e)
    {
        std::cerr << "Exception occured during action extraction:" << std::endl << e.what() << std::endl;
        return -1;
    }

    time_elapsed = duration_cast<microseconds>(high_resolution_clock::now() - start_time);

    std::cout << "Finished action extraction (" << time_elapsed.count() << " μs)" << std::endl;


    std::cout << "Beginning action selection, scene duplication, and action sampling/replacement" << std::endl;

    start_time = high_resolution_clock::now();

    structures::IArray<agent::IDrivingAgent const*> *driving_agents_with_actions =
            scene_with_actions->get_driving_agents();

    agent::IEvent<FP_DATA_TYPE> const *selected_action_goal_value_event = nullptr;
    agent::IEvent<temporal::Time> const *selected_action_goal_time_event = nullptr;
    temporal::Time previous_action_time, next_action_time;

    for (i = 0; i < driving_agents_with_actions->count(); ++i)
    {
        agent::IDrivingAgent const *driving_agent_with_actions =
                (*driving_agents_with_actions)[i];

        agent::IValuelessVariable const *aligned_linear_velocity_goal_value_valueless_variable =
                driving_agent_with_actions->get_variable_parameter(
                    driving_agent_with_actions->get_name() +
                    ".aligned_linear_velocity.goal_value");
        agent::IValuelessVariable const *aligned_linear_velocity_goal_time_valueless_variable =
                driving_agent_with_actions->get_variable_parameter(
                    driving_agent_with_actions->get_name() +
                    ".aligned_linear_velocity.goal_time");

        agent::IVariable<FP_DATA_TYPE> const *aligned_linear_velocity_goal_value_variable =
                dynamic_cast<agent::IVariable<FP_DATA_TYPE> const*>(
                    aligned_linear_velocity_goal_value_valueless_variable);
        agent::IVariable<temporal::Time> const *aligned_linear_velocity_goal_time_variable =
                dynamic_cast<agent::IVariable<temporal::Time> const*>(
                    aligned_linear_velocity_goal_time_valueless_variable);

        structures::IArray<agent::IEvent<FP_DATA_TYPE> const*> *aligned_linear_velocity_goal_value_events =
                aligned_linear_velocity_goal_value_variable->get_events();

        for (j = 0; j < aligned_linear_velocity_goal_value_events->count(); ++j)
        {
            agent::IEvent<FP_DATA_TYPE> const *aligned_linear_velocity_goal_value_event =
                    (*aligned_linear_velocity_goal_value_events)[j];

            if (aligned_linear_velocity_goal_value_event->get_time() >
                    driving_agent_with_actions->get_min_temporal_limit())
            {
                selected_action_goal_value_event = aligned_linear_velocity_goal_value_event;
                selected_action_goal_time_event =
                        aligned_linear_velocity_goal_time_variable->get_event(
                            selected_action_goal_value_event->get_time(),
                            true);
                if (selected_action_goal_time_event != nullptr)
                {
                    if (j == 0)
                    {
                        previous_action_time =
                                driving_agent_with_actions->get_min_temporal_limit();
                    }
                    else
                    {
                        previous_action_time =
                                (*aligned_linear_velocity_goal_value_events)[j - 1]->get_time();
                    }
                    if (j == aligned_linear_velocity_goal_value_events->count() - 1)
                    {
                        next_action_time =
                                driving_agent_with_actions->get_max_temporal_limit();
                    }
                    else
                    {
                        next_action_time =
                                (*aligned_linear_velocity_goal_value_events)[j + 1]->get_time();
                    }
                    break;
                }
                else
                {
                    selected_action_goal_value_event = nullptr;
                    continue;
                }
            }
        }

        delete aligned_linear_velocity_goal_value_events;

        if (selected_action_goal_value_event != nullptr)
        {
            break;
        }
    }

    delete driving_agents_with_actions;

    if (selected_action_goal_value_event == nullptr)
    {
        std::cerr << "Could not find action to replace" << std::endl;
        return -1;
    }

    std::random_device random_device;
    std::mt19937 randomness_generator(random_device());
    std::uniform_int_distribution<size_t> agent_selector(1, NUMBER_OF_AGENTS);

    structures::ISet<std::string> *simulated_agent_names = new structures::stl::STLSet<std::string>;

    simulated_agent_names->insert(selected_action_goal_value_event->get_entity_name());

    for (i = 1; i < NUMBER_OF_SIMULATED_AGENTS; ++i)
    {
        size_t agent_id = agent_selector(randomness_generator);
        std::string agent_name = "non_ego_vehicle_" + std::to_string(agent_id);
        if (!simulated_agent_names->contains(agent_name))
        {
            simulated_agent_names->insert(agent_name);
        }
        else
        {
            --i;
        }
    }

    agent::BasicFPActionSampler action_sampler;

    structures::IArray<agent::IDrivingScene*> *scenes_with_actions =
            new structures::stl::STLStackArray<agent::IDrivingScene*>(NUMBER_OF_SCENES);

    (*scenes_with_actions)[0] = scene_with_actions;

    for (i = 1; i < NUMBER_OF_SCENES; ++i)
    {
        agent::IDrivingScene *new_scene = scene_with_actions->driving_scene_deep_copy();

        agent::IDrivingAgent *driving_agent_to_edit =
                new_scene->get_mutable_driving_agent(
                    selected_action_goal_value_event->get_entity_name());

        agent::IValuelessVariable *aligned_linear_velocity_goal_value_valueless_variable =
                driving_agent_to_edit->get_mutable_variable_parameter(
                    selected_action_goal_value_event->get_full_name());
        agent::IValuelessVariable *aligned_linear_velocity_goal_time_valueless_variable =
                driving_agent_to_edit->get_mutable_variable_parameter(
                    selected_action_goal_time_event->get_full_name());

        agent::IVariable<FP_DATA_TYPE> *aligned_linear_velocity_goal_value_variable =
                dynamic_cast<agent::IVariable<FP_DATA_TYPE>*>(
                    aligned_linear_velocity_goal_value_valueless_variable);
        agent::IVariable<temporal::Time> *aligned_linear_velocity_goal_time_variable =
                dynamic_cast<agent::IVariable<temporal::Time>*>(
                    aligned_linear_velocity_goal_time_valueless_variable);

        FP_DATA_TYPE new_goal_value;
        temporal::Time new_action_start_time;
        temporal::Time new_action_end_time;

        action_sampler.sample_action(previous_action_time,
                                     next_action_time,
                                     MIN_ALIGNED_LINEAR_VELOCITY,
                                     MAX_ALIGNED_LINEAR_VELOCITY,
                                     new_goal_value,
                                     new_action_start_time,
                                     new_action_end_time);

        aligned_linear_velocity_goal_value_variable->remove_value(
                    selected_action_goal_value_event->get_time());
        aligned_linear_velocity_goal_time_variable->remove_value(
                    selected_action_goal_time_event->get_time());

        aligned_linear_velocity_goal_value_variable->set_value(new_action_start_time,
                                                               new_goal_value);
        aligned_linear_velocity_goal_time_variable->set_value(new_action_start_time,
                                                              new_action_end_time);

        (*scenes_with_actions)[i] = new_scene;
    }

    time_elapsed = duration_cast<microseconds>(high_resolution_clock::now() - start_time);

    std::cout << "Finished action selection, scene duplication, and action sampling/replacement (" << time_elapsed.count() << " μs)" << std::endl;


    temporal::Time simulation_start_time = scene->get_min_temporal_limit() +
            temporal::Duration(1000);

    temporal::Duration time_step(40);

    agent::IDrivingAgentController *driving_agent_controller =
                new agent::BasicDrivingAgentController<uint8_t>(map, time_step, 10);

    agent::IDrivingSimulator *driving_simulator =
                new agent::BasicDrivingSimulator(driving_agent_controller);

    structures::IArray<agent::IDrivingSimulationScene*> *simulated_scenes =
            new structures::stl::STLStackArray<agent::IDrivingSimulationScene*>(NUMBER_OF_SCENES);

    for (i = 0; i < NUMBER_OF_SCENES; ++i)
    {
        (*simulated_scenes)[i] = agent::DrivingSimulationScene::construct_from(
                    (*scenes_with_actions)[i], driving_simulator, time_step,
                    simulation_start_time, simulated_agent_names);
    }

    delete simulated_agent_names;

    std::cout << "Beginning simulation" << std::endl;

    start_time = high_resolution_clock::now();

    for (i = 0; i < NUMBER_OF_SCENES; ++i)
    {
        simulate((*simulated_scenes)[i]);
    }

    time_elapsed = duration_cast<microseconds>(high_resolution_clock::now() - start_time);

    temporal::Duration simulation_time_covered = scene->get_max_temporal_limit() - simulation_start_time;
    float real_time_factor =
            NUMBER_OF_SCENES *
            float(duration_cast<microseconds>(simulation_time_covered).count()) /
            float(time_elapsed.count());

    std::cout << "Finished simulation (" << time_elapsed.count() << " μs, rtf = " << real_time_factor << ")" << std::endl;

    for (i = 0; i < NUMBER_OF_SCENES; ++i)
    {
        delete (*simulated_scenes)[i];
        delete (*scenes_with_actions)[i];
    }
    delete simulated_scenes;
    delete scenes_with_actions;

    delete scene;

    delete driving_simulator;
    delete driving_agent_controller;

    delete map;

    geometry::TrigBuff::destroy_instance();
}
