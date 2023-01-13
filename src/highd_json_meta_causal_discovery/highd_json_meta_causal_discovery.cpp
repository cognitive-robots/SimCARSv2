
#include <ori/simcars/structures/stl/stl_set.hpp>
#include <ori/simcars/geometry/trig_buff.hpp>
#include <ori/simcars/map/highd/highd_map.hpp>
#include <ori/simcars/agent/highd/highd_scene.hpp>
#include <ori/simcars/causal/necessary_driving_causal_discoverer.hpp>

#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/ostreamwrapper.h>
#include <rapidjson/writer.h>

#include <iostream>
#include <exception>
#include <string>
#include <fstream>
#include <filesystem>

#define CONTROLLER_LOOKAHEAD_STEPS 10

using namespace ori::simcars;
using namespace std::chrono;

int main(int argc, char *argv[])
{
    if (argc < 5)
    {
        std::cerr << "Usage: ./highd_json_meta_causal_discovery ate_threshold branch_count input_json_meta_file_path raw_data_directory_path [output_json_meta_file_path]" << std::endl;
        return -1;
    }

    FP_DATA_TYPE ate_threshold = std::atof(argv[1]);
    size_t branch_count = std::atoi(argv[2]);

    std::cout << "ATE Threshold: " << std::to_string(ate_threshold) << std::endl;
    std::cout << "Branch Count: " << std::to_string(branch_count) << std::endl;

    std::string input_json_meta_file_path_str = argv[3];
    std::filesystem::path input_json_meta_file_path(input_json_meta_file_path_str);

    if (!std::filesystem::is_regular_file(input_json_meta_file_path))
    {
        throw std::invalid_argument("Input JSON meta file path '" +
                                    input_json_meta_file_path_str +
                                    "' does not indicate a valid file");
    }

    std::string output_json_meta_file_path_str;
    std::filesystem::path output_json_meta_file_path;

    if (argc > 5)
    {
        output_json_meta_file_path_str = argv[5];
        output_json_meta_file_path = std::filesystem::path(output_json_meta_file_path_str);
        std::filesystem::path output_json_meta_file_path_dir = output_json_meta_file_path.parent_path();

        if (!std::filesystem::is_directory(output_json_meta_file_path_dir) ||
                std::filesystem::is_directory(output_json_meta_file_path))
        {
            throw std::invalid_argument("Output JSON meta file path '" +
                                        output_json_meta_file_path_str +
                                        "' does not indicate a valid location to output a file");
        }
    }

    std::ifstream input_json_meta_filestream(input_json_meta_file_path);
    rapidjson::IStreamWrapper input_json_meta_stream(input_json_meta_filestream);

    rapidjson::Document json_meta_document;
    json_meta_document.ParseStream(input_json_meta_stream);

    uint32_t scene_id = json_meta_document["scene_id"].GetUint();
    uint32_t convoy_head_id = json_meta_document["convoy_head_id"].GetUint();
    uint32_t convoy_tail_id = json_meta_document["convoy_tail_id"].GetUint();
    uint32_t independent_id = json_meta_document["independent_id"].GetUint();

    std::string convoy_head_str = "non_ego_vehicle_" + std::to_string(convoy_head_id);
    std::string convoy_tail_str = "non_ego_vehicle_" + std::to_string(convoy_tail_id);
    std::string independent_str = "non_ego_vehicle_" + std::to_string(independent_id);

    std::cout << "Convoy Head Agent = " << convoy_head_str << std::endl;
    std::cout << "Convoy Tail Agent = " << convoy_tail_str << std::endl;
    std::cout << "Independent Agent = " << independent_str << std::endl;

    std::string raw_data_directory_path_str = argv[4];
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
                map, scene->get_time_step(), CONTROLLER_LOOKAHEAD_STEPS, ate_threshold,
                branch_count);

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


    if (argc > 5)
    {
        rapidjson::Value json_causal_links(rapidjson::kObjectType);
        for (i = 0; i < entity_causal_link_array->count(); ++i)
        {
            std::pair<std::string, std::string> const &entity_causal_link =
                    (*entity_causal_link_array)[i];
            rapidjson::Value cause_str;
            cause_str = rapidjson::StringRef(entity_causal_link.first.c_str());
            rapidjson::Value effect_str;
            effect_str = rapidjson::StringRef(entity_causal_link.second.c_str());
            if (!json_causal_links.HasMember(cause_str))
            {
                rapidjson::Value json_causal_effects(rapidjson::kArrayType);
                json_causal_effects.PushBack(effect_str, json_meta_document.GetAllocator());
                json_causal_links.AddMember(cause_str, json_causal_effects,
                                            json_meta_document.GetAllocator());
            }
            else
            {
                rapidjson::Value::Array const &json_causal_effects =
                        json_causal_links[cause_str].GetArray();
                json_causal_effects.PushBack(effect_str, json_meta_document.GetAllocator());
            }
        }
        json_meta_document.AddMember("causal_links", json_causal_links,
                                     json_meta_document.GetAllocator());

        std::ofstream output_json_meta_filestream(output_json_meta_file_path);
        rapidjson::OStreamWrapper output_json_meta_stream(output_json_meta_filestream);
        rapidjson::Writer<rapidjson::OStreamWrapper> output_json_meta_writer(
                    output_json_meta_stream);
        json_meta_document.Accept(output_json_meta_writer);
    }


    delete entity_causal_links;

    delete agents_of_interest;

    delete causal_discoverer;

    delete scene;

    delete map;

    geometry::TrigBuff::destroy_instance();
}
