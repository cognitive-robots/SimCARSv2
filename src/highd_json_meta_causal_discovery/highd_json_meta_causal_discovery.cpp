
#include <ori/simcars/structures/stl/stl_set.hpp>
#include <ori/simcars/geometry/trig_buff.hpp>
#include <ori/simcars/map/highd/highd_map.hpp>
#include <ori/simcars/agent/highd/highd_scene.hpp>
#include <ori/simcars/causal/necessary_driving_causal_discoverer.hpp>

#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>

#include <iostream>
#include <exception>
#include <string>
#include <fstream>
#include <filesystem>

#define ATE_THRESHOLD 0.05f
#define BRANCH_COUNT 100
#define CONTROLLER_LOOKAHEAD_STEPS 10

using namespace ori::simcars;
using namespace std::chrono;

int main(int argc, char *argv[])
{
    if (argc < 3)
    {
        std::cerr << "Usage: ./highd_json_meta_ate_test json_meta_file_path raw_data_directory_path" << std::endl;
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

    std::cout << "Convoy Head Agent = non_ego_vehicle_" + std::to_string(convoy_head_id) <<
                 std::endl;
    std::cout << "Convoy Tail Agent = non_ego_vehicle_" + std::to_string(convoy_tail_id) <<
                 std::endl;
    std::cout << "Independent Agent = non_ego_vehicle_" + std::to_string(independent_id) <<
                 std::endl;

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

    structures::ISet<std::string>* agents_of_interest = new structures::stl::STLSet<std::string>;
    agents_of_interest->insert("non_ego_vehicle_" + std::to_string(convoy_head_id));
    agents_of_interest->insert("non_ego_vehicle_" + std::to_string(convoy_tail_id));
    agents_of_interest->insert("non_ego_vehicle_" + std::to_string(independent_id));


    geometry::TrigBuff::init_instance(360000, geometry::AngleType::RADIANS);

    time_point<high_resolution_clock> start_time;
    microseconds time_elapsed;

    std::cout << "Beginning map load" << std::endl;

    start_time = high_resolution_clock::now();

    map::IMap<uint8_t> const *map = map::highd::HighDMap::load(recording_meta_file_path.string());

    time_elapsed = duration_cast<microseconds>(high_resolution_clock::now() - start_time);

    std::cout << "Finished map load (" << time_elapsed.count() << " μs)" << std::endl;

    std::cout << "Beginning scene load" << std::endl;

    start_time = high_resolution_clock::now();

    agent::IDrivingScene *scene = agent::highd::HighDScene::load(tracks_meta_file_path.string(),
                                                                 tracks_file_path.string());

    time_elapsed = duration_cast<microseconds>(high_resolution_clock::now() - start_time);

    std::cout << "Finished scene load (" << time_elapsed.count() << " μs)" << std::endl;

    std::cout << "Beginning causal discovery" << std::endl;

    start_time = high_resolution_clock::now();

    causal::ICausalDiscoverer *causal_discoverer = new causal::NecessaryDrivingCausalDiscoverer(
                map, scene->get_time_step(), CONTROLLER_LOOKAHEAD_STEPS, ATE_THRESHOLD,
                BRANCH_COUNT);

    structures::ISet<std::pair<std::string, std::string>> *entity_causal_links =
            causal_discoverer->discover_entity_causal_links(scene, agents_of_interest);

    time_elapsed = duration_cast<microseconds>(high_resolution_clock::now() - start_time);

    std::cout << "Finished causal discovery (" << time_elapsed.count() << " μs)" << std::endl;


    structures::IArray<std::pair<std::string, std::string>> const *entity_causal_link_array =
            entity_causal_links->get_array();

    std::cout << "Discovered Causal Links: " << std::endl;
    size_t i;
    for (i = 0; i < entity_causal_link_array->count(); ++i)
    {
        std::pair<std::string, std::string> const &entity_causal_link =
                (*entity_causal_link_array)[i];
        std::cout << entity_causal_link.first << " -> " << entity_causal_link.second << std::endl;
    }


    delete entity_causal_links;

    delete agents_of_interest;

    delete causal_discoverer;

    delete scene;

    delete map;

    geometry::TrigBuff::destroy_instance();
}
