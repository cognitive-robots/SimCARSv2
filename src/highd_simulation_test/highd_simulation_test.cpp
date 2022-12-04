
#include <ori/simcars/structures/stl/stl_set.hpp>
#include <ori/simcars/geometry/trig_buff.hpp>
#include <ori/simcars/map/highd/highd_map.hpp>
#include <ori/simcars/agent/driving_goal_extraction_scene.hpp>
#include <ori/simcars/agent/basic_driving_agent_controller.hpp>
#include <ori/simcars/agent/basic_driving_simulator.hpp>
#include <ori/simcars/agent/driving_simulation_scene.hpp>
#include <ori/simcars/agent/highd/highd_scene.hpp>

#include <iostream>
#include <exception>
#include <random>

#define NUMBER_OF_AGENTS 10
#define NUMBER_OF_SIMULATED_AGENTS 2

using namespace ori::simcars;
using namespace std::chrono;

void simulate(agent::IDrivingScene const *simulated_scene)
{
    structures::IArray<agent::IDrivingAgent const*> *driving_agents =
            simulated_scene->get_driving_agents();

    for (size_t i = 0; i < driving_agents->count(); ++i)
    {
        try
        {
            agent::IDrivingAgent const *driving_agent = (*driving_agents)[i];
            agent::IVariable<geometry::Vec> const *position_variable =
                    driving_agent->get_position_variable();
            position_variable->get_value(simulated_scene->get_max_temporal_limit());
        }
        catch (std::out_of_range)
        {
            //std::cerr << "Position not available for this time" << std::endl;
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

    size_t i;
    for (i = 1; i <= NUMBER_OF_AGENTS; ++i)
    {
        agent_names->insert("non_ego_vehicle_" + std::to_string(i));
    }

    std::random_device random_device;
    std::mt19937 randomness_generator(random_device());
    std::uniform_int_distribution<size_t> agent_selector(1, NUMBER_OF_AGENTS);

    structures::ISet<std::string> *simulated_agent_names = new structures::stl::STLSet<std::string>;

    for (i = 0; i < NUMBER_OF_SIMULATED_AGENTS; ++i)
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

    std::cout << "Finished map load (" << time_elapsed.count() << " us)" << std::endl;

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

    std::cout << "Finished scene load (" << time_elapsed.count() << " us)" << std::endl;

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

    std::cout << "Finished action extraction (" << time_elapsed.count() << " us)" << std::endl;

    temporal::Time simulation_start_time = scene->get_min_temporal_limit() +
            temporal::Duration(1000);

    temporal::Duration time_step(40);

    agent::IDrivingAgentController *driving_agent_controller =
                new agent::BasicDrivingAgentController<uint8_t>(map, time_step, 10);

    agent::IDrivingSimulator *driving_simulator =
                new agent::BasicDrivingSimulator(driving_agent_controller);

    agent::IDrivingScene *simulated_scene =
            agent::DrivingSimulationScene::construct_from(
                scene_with_actions, driving_simulator, time_step,
                simulation_start_time, simulated_agent_names);

    delete simulated_agent_names;

    std::cout << "Beginning simulation" << std::endl;

    start_time = high_resolution_clock::now();

    simulate(simulated_scene);

    time_elapsed = duration_cast<microseconds>(high_resolution_clock::now() - start_time);

    temporal::Duration simulation_time_covered = scene->get_max_temporal_limit() - simulation_start_time;
    float real_time_factor = float(duration_cast<microseconds>(simulation_time_covered).count()) / float(time_elapsed.count());

    std::cout << "Finished simulation (" << time_elapsed.count() << " us, rtf = " << real_time_factor << ")" << std::endl;

    delete simulated_scene;
    delete scene_with_actions;
    delete scene;

    delete map;

    geometry::TrigBuff::destroy_instance();
}
