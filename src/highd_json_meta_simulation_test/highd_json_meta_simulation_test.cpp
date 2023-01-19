
#include <ori/simcars/structures/stl/stl_set.hpp>
#include <ori/simcars/geometry/trig_buff.hpp>
#include <ori/simcars/map/highd/highd_map.hpp>
#include <ori/simcars/agent/driving_goal_extraction_scene.hpp>
#include <ori/simcars/agent/basic_driving_agent_controller.hpp>
#include <ori/simcars/agent/basic_driving_simulator.hpp>
#include <ori/simcars/agent/driving_simulation_scene.hpp>
#include <ori/simcars/agent/highd/highd_scene.hpp>

#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>

#include <iostream>
#include <exception>
#include <random>
#include <string>
#include <fstream>
#include <filesystem>

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
        temporal::Time last_event_time = position_variable->get_last_event_time();
        geometry::Vec position;
        if (position_variable->get_value(last_event_time, position))
        {
            std::cout << "Got position variable value for agent " <<
                         i + 1 << " at end of scene" << std::endl;
        }
    }

    delete driving_agents;
}

int main(int argc, char *argv[])
{
    if (argc < 3)
    {
        std::cerr << "Usage: ./highd_json_meta_simulation_test json_meta_file_path trimmed_data_directory_path" << std::endl;
        return -1;
    }

    std::string json_meta_file_path_str = argv[1];
    std::filesystem::path json_meta_file_path(json_meta_file_path_str);

    if (!std::filesystem::is_regular_file(json_meta_file_path))
    {
        throw std::invalid_argument("JSON meta file path '" +
                                    json_meta_file_path_str +
                                    "' does not indicate a valid file");
    }

    std::ifstream json_meta_filestream(json_meta_file_path);
    rapidjson::BasicIStreamWrapper json_meta_stream(json_meta_filestream);

    rapidjson::Document json_meta_document;
    json_meta_document.ParseStream(json_meta_stream);

    uint32_t scene_id = json_meta_document["scene_id"].GetUint();
    uint32_t convoy_head_id = json_meta_document["convoy_head_id"].GetUint();
    uint32_t convoy_tail_id = json_meta_document["convoy_tail_id"].GetUint();
    uint32_t independent_id = json_meta_document["independent_id"].GetUint();

    std::string raw_data_directory_path_str = argv[2];
    std::filesystem::path raw_data_directory_path(raw_data_directory_path_str);

    std::filesystem::path recording_meta_file_path =
            raw_data_directory_path /
                ("scene-" + std::to_string(scene_id) + "-" + std::to_string(convoy_tail_id) +
                 "_follows_" + std::to_string(convoy_head_id) + "-" +
                 std::to_string(independent_id) + "_independent-recordingMeta.csv");

    if (!std::filesystem::is_regular_file(recording_meta_file_path))
    {
        throw std::invalid_argument("Recording meta file path '" +
                                    recording_meta_file_path.string() +
                                    "' does not indicate a valid file");
    }

    std::filesystem::path tracks_meta_file_path =
            raw_data_directory_path /
                ("scene-" + std::to_string(scene_id) + "-" + std::to_string(convoy_tail_id) +
                 "_follows_" + std::to_string(convoy_head_id) + "-" +
                 std::to_string(independent_id) + "_independent-tracksMeta.csv");

    if (!std::filesystem::is_regular_file(tracks_meta_file_path))
    {
        throw std::invalid_argument("Tracks meta file path '" +
                                    tracks_meta_file_path.string() +
                                    "' does not indicate a valid file");
    }

    std::filesystem::path tracks_file_path =
            raw_data_directory_path /
                ("scene-" + std::to_string(scene_id) + "-" + std::to_string(convoy_tail_id) +
                 "_follows_" + std::to_string(convoy_head_id) + "-" +
                 std::to_string(independent_id) + "_independent-tracks.csv");

    if (!std::filesystem::is_regular_file(tracks_file_path))
    {
        throw std::invalid_argument("Tracks file path '" +
                                    tracks_file_path.string() +
                                    "' does not indicate a valid file");
    }

    structures::ISet<std::string> *simulated_agent_names =
            new structures::stl::STLSet<std::string>;
    simulated_agent_names->insert("non_ego_vehicle_" + std::to_string(convoy_head_id));
    simulated_agent_names->insert("non_ego_vehicle_" + std::to_string(convoy_tail_id));


    geometry::TrigBuff::init_instance(360000, geometry::AngleType::RADIANS);

    time_point<high_resolution_clock> start_time;
    microseconds time_elapsed;

    std::cout << "Beginning map load" << std::endl;

    start_time = high_resolution_clock::now();

    map::IMap<uint8_t> const *map;

    try
    {
        map = map::highd::HighDMap::load(recording_meta_file_path.string());
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
        scene = agent::highd::HighDScene::load(tracks_meta_file_path.string(),
                                               tracks_file_path.string());
    }
    catch (std::exception const &e)
    {
        std::cerr << "Exception occured during scene load:" << std::endl << e.what() << std::endl;
        return -1;
    }

    time_elapsed = duration_cast<microseconds>(high_resolution_clock::now() - start_time);

    std::cout << "Finished scene load (" << time_elapsed.count() << " μs)" << std::endl;


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


    temporal::Time simulation_start_time =
            scene_with_actions->get_min_temporal_limit() +
            temporal::Duration(int64_t(0.25 *
            (scene_with_actions->get_max_temporal_limit() -
             scene_with_actions->get_min_temporal_limit()).count()));
    temporal::Time simulation_end_time =
            scene_with_actions->get_min_temporal_limit() +
            temporal::Duration(int64_t(0.75 *
            (scene_with_actions->get_max_temporal_limit() -
             scene_with_actions->get_min_temporal_limit()).count()));

    temporal::Duration time_step(40);

    agent::IDrivingAgentController *driving_agent_controller =
                new agent::BasicDrivingAgentController<uint8_t>(map, time_step, 10);

    agent::IDrivingSimulator *driving_simulator =
                new agent::BasicDrivingSimulator(driving_agent_controller);

    agent::IDrivingScene *simulated_scene =
            agent::DrivingSimulationScene::construct_from(
                scene_with_actions, driving_simulator, time_step,
                simulation_start_time, simulation_end_time,
                simulated_agent_names);

    delete simulated_agent_names;

    std::cout << "Beginning simulation" << std::endl;

    start_time = high_resolution_clock::now();

    simulate(simulated_scene);

    time_elapsed = duration_cast<microseconds>(high_resolution_clock::now() - start_time);

    temporal::Duration simulation_time_covered = scene->get_max_temporal_limit() - simulation_start_time;
    float real_time_factor = float(duration_cast<microseconds>(simulation_time_covered).count()) / float(time_elapsed.count());

    std::cout << "Finished simulation (" << time_elapsed.count() << " μs, rtf = " << real_time_factor << ")" << std::endl;

    delete simulated_scene;
    delete scene_with_actions;
    delete scene;

    delete driving_simulator;
    delete driving_agent_controller;

    delete map;

    geometry::TrigBuff::destroy_instance();
}
