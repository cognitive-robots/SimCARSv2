
#include <ori/simcars/structures/stl/stl_set.hpp>
#include <ori/simcars/geometry/trig_buff.hpp>
#include <ori/simcars/map/highd/highd_map.hpp>
#include <ori/simcars/agent/driving_goal_extraction_scene.hpp>
#include <ori/simcars/agent/basic_fp_action_sampler.hpp>
#include <ori/simcars/agent/basic_driving_agent_controller.hpp>
#include <ori/simcars/agent/basic_driving_simulator.hpp>
#include <ori/simcars/agent/driving_simulation_scene.hpp>
#include <ori/simcars/agent/highd/highd_scene.hpp>
#include <ori/simcars/visualisation/qmap_scene_widget.hpp>

#include <QApplication>
#include <QFrame>
#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidcsv.h>

#include <iostream>
#include <exception>
#include <random>
#include <string>
#include <fstream>
#include <filesystem>

using namespace ori::simcars;
using namespace std::chrono;

int main(int argc, char *argv[])
{
    if (argc < 3)
    {
        std::cerr << "Usage: ./highd_json_meta_simcars_intervention_demo json_meta_file_path raw_data_directory_path" << std::endl;
        return -1;
    }

    QApplication app(argc, argv);

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
                (std::to_string(scene_id) + "_recordingMeta.csv");

    if (!std::filesystem::is_regular_file(recording_meta_file_path))
    {
        throw std::invalid_argument("Recording meta file path '" +
                                    recording_meta_file_path.string() +
                                    "' does not indicate a valid file");
    }

    std::filesystem::path tracks_meta_file_path =
            raw_data_directory_path /
                (std::to_string(scene_id) + "_tracksMeta.csv");

    if (!std::filesystem::is_regular_file(tracks_meta_file_path))
    {
        throw std::invalid_argument("Tracks meta file path '" +
                                    tracks_meta_file_path.string() +
                                    "' does not indicate a valid file");
    }

    std::filesystem::path tracks_file_path =
            raw_data_directory_path /
                (std::to_string(scene_id) + "_tracks.csv");

    if (!std::filesystem::is_regular_file(tracks_file_path))
    {
        throw std::invalid_argument("Tracks file path '" +
                                    tracks_file_path.string() +
                                    "' does not indicate a valid file");
    }

    std::ifstream tracks_meta_filestream(tracks_meta_file_path);
    rapidcsv::Document tracks_meta_document(tracks_meta_filestream);

    uint32_t latest_initial_frame =
            tracks_meta_document.GetCell<uint32_t>("initialFrame", convoy_head_id - 1);
    latest_initial_frame =
            std::max(tracks_meta_document.GetCell<uint32_t>("initialFrame",
                                                            convoy_tail_id - 1),
                     latest_initial_frame);
    latest_initial_frame =
            std::max(tracks_meta_document.GetCell<uint32_t>("initialFrame",
                                                            independent_id - 1),
                     latest_initial_frame);

    uint32_t earliest_final_frame =
            tracks_meta_document.GetCell<uint32_t>("finalFrame", convoy_head_id - 1);
    earliest_final_frame =
            std::min(tracks_meta_document.GetCell<uint32_t>("finalFrame",
                                                            convoy_tail_id - 1),
                     earliest_final_frame);
    earliest_final_frame =
            std::min(tracks_meta_document.GetCell<uint32_t>("finalFrame",
                                                            independent_id - 1),
                     earliest_final_frame);


    structures::ISet<std::string> *agent_names =
            new structures::stl::STLSet<std::string>;

    size_t i;
    for (i = 1; i <= tracks_meta_document.GetRowCount(); ++i)
    {
        uint32_t const initial_frame =
                tracks_meta_document.GetCell<uint32_t>("initialFrame", i - 1);
        uint32_t const final_frame =
                tracks_meta_document.GetCell<uint32_t>("finalFrame", i - 1);

        if (initial_frame <= earliest_final_frame &&
                final_frame >= latest_initial_frame)
        {
            agent_names->insert("non_ego_vehicle_" + std::to_string(i));
        }
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
                                               tracks_file_path.string(), agent_names);
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

    agent::IEvent<agent::Goal<FP_DATA_TYPE>> const *selected_action_goal_event = nullptr;
    temporal::Time previous_action_time, next_action_time;

    agent::IDrivingAgent const *driving_agent_with_actions =
            scene_with_actions->get_driving_agent(
                "non_ego_vehicle_" + std::to_string(convoy_head_id));

    agent::IValuelessVariable const *aligned_linear_velocity_goal_valueless_variable =
            driving_agent_with_actions->get_variable_parameter(
                driving_agent_with_actions->get_name() +
                ".aligned_linear_velocity.goal");

    agent::IVariable<agent::Goal<FP_DATA_TYPE>> const *aligned_linear_velocity_goal_variable =
            dynamic_cast<agent::IVariable<agent::Goal<FP_DATA_TYPE>> const*>(
                                                                         aligned_linear_velocity_goal_valueless_variable);

    structures::IArray<agent::IEvent<agent::Goal<FP_DATA_TYPE>> const*> *aligned_linear_velocity_goal_events =
            aligned_linear_velocity_goal_variable->get_events();

    if (aligned_linear_velocity_goal_events->count() == 0)
    {
        std::cerr << "Could not find action to replace" << std::endl;
        return -1;
    }

    size_t selected_action_goal_event_index;
    if (aligned_linear_velocity_goal_events->count() > 2)
    {
        selected_action_goal_event_index =
                aligned_linear_velocity_goal_events->count() - 2;
    }
    else
    {
        selected_action_goal_event_index =
                aligned_linear_velocity_goal_events->count() - 1;
    }

    selected_action_goal_event =
            (*aligned_linear_velocity_goal_events)[selected_action_goal_event_index];

    if (aligned_linear_velocity_goal_events->count() < 2)
    {
        previous_action_time =
                driving_agent_with_actions->get_min_temporal_limit();
    }
    else
    {
        previous_action_time =
                (*aligned_linear_velocity_goal_events)[selected_action_goal_event_index - 1]->get_time();
    }
    if (aligned_linear_velocity_goal_events->count() < 3)
    {
        next_action_time =
                driving_agent_with_actions->get_max_temporal_limit();
    }
    else
    {
        next_action_time =
                (*aligned_linear_velocity_goal_events)[selected_action_goal_event_index + 1]->get_time();
    }

    delete aligned_linear_velocity_goal_events;

    agent::BasicFPActionSampler action_sampler;

    agent::IDrivingScene *intervened_scene = scene_with_actions->driving_scene_deep_copy();

    agent::IDrivingAgent *driving_agent_to_edit =
            intervened_scene->get_mutable_driving_agent(
                selected_action_goal_event->get_entity_name());

    agent::IValuelessVariable *intervened_aligned_linear_velocity_goal_valueless_variable =
            driving_agent_to_edit->get_mutable_variable_parameter(
                selected_action_goal_event->get_full_name());

    agent::IVariable<agent::Goal<FP_DATA_TYPE>> *intervened_aligned_linear_velocity_goal_variable =
            dynamic_cast<agent::IVariable<agent::Goal<FP_DATA_TYPE>>*>(intervened_aligned_linear_velocity_goal_valueless_variable);

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

    if (selected_action_goal_event->get_time() == driving_agent_to_edit->get_min_temporal_limit())
    {
        agent::IVariable<FP_DATA_TYPE> const *aligned_linear_velocity_variable =
                driving_agent_to_edit->get_aligned_linear_velocity_variable();
        FP_DATA_TYPE starting_aligned_linear_velocity;
        bool result = aligned_linear_velocity_variable->get_value(
                    driving_agent_to_edit->get_min_temporal_limit(),
                    starting_aligned_linear_velocity);
        if (!result)
        {
            std::cerr << "Could not find starting velocity to create replacement starting velocity goal from" << std::endl;
            return -1;
        }
        intervened_aligned_linear_velocity_goal_variable->set_value(
                    selected_action_goal_event->get_time(),
                    agent::Goal(starting_aligned_linear_velocity,
                                selected_action_goal_event->get_time()));
    }
    else
    {
        intervened_aligned_linear_velocity_goal_variable->remove_value(
                    selected_action_goal_event->get_time());
    }

    intervened_aligned_linear_velocity_goal_variable->set_value(
                new_action_start_time,
                agent::Goal(new_goal_value,
                            new_action_end_time));

    intervened_aligned_linear_velocity_goal_variable->propogate_events_forward(next_action_time);

    assert(intervened_aligned_linear_velocity_goal_variable->get_max_temporal_limit() >= next_action_time);
    assert(intervened_aligned_linear_velocity_goal_variable->get_event(intervened_aligned_linear_velocity_goal_variable->get_max_temporal_limit()) != nullptr);

    time_elapsed = duration_cast<microseconds>(high_resolution_clock::now() - start_time);

    std::cout << "Finished action selection, scene duplication, and action sampling/replacement (" << time_elapsed.count() << " μs)" << std::endl;


    temporal::Time simulation_start_time = previous_action_time;
    temporal::Time simulation_end_time = next_action_time;

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

    agent::IDrivingScene *simulated_intervened_scene =
            agent::DrivingSimulationScene::construct_from(
                intervened_scene, driving_simulator, time_step,
                simulation_start_time, simulation_end_time,
                simulated_agent_names);

    delete simulated_agent_names;

    std::cout << "Beginning simulation" << std::endl;

    start_time = high_resolution_clock::now();

    QFrame *frame = new QFrame();
    frame->setWindowTitle("SIMCARS Demo");
    frame->setFixedSize(2400, 800);
    frame->show();

    geometry::Vec focal_position(210.0f, 19.0f);

    visualisation::QMapSceneWidget<uint8_t> *map_scene_widget =
                new visualisation::QMapSceneWidget<uint8_t>(
                    map,
                    scene,
                    frame,
                    QPoint(20, 20),
                    QSize(760, 760),
                    1.0f,
                    30.0f,
                    1.0f,
                    2.0f,
                    false);
    map_scene_widget->set_focal_position(focal_position);
    map_scene_widget->show();

    visualisation::QMapSceneWidget<uint8_t> *map_simulated_scene_widget =
                new visualisation::QMapSceneWidget<uint8_t>(
                    map,
                    simulated_scene,
                    frame,
                    QPoint(820, 20),
                    QSize(760, 760),
                    1.0f,
                    30.0f,
                    1.0f,
                    2.0f,
                    false);
    map_simulated_scene_widget->set_focal_position(focal_position);
    map_simulated_scene_widget->show();

    visualisation::QMapSceneWidget<uint8_t> *map_simulated_intervened_scene_widget =
                new visualisation::QMapSceneWidget<uint8_t>(
                    map,
                    simulated_intervened_scene,
                    frame,
                    QPoint(1620, 20),
                    QSize(760, 760),
                    1.0f,
                    30.0f,
                    1.0f,
                    2.0f,
                    false);
    map_simulated_intervened_scene_widget->set_focal_position(focal_position);
    map_simulated_intervened_scene_widget->show();

    int result = app.exec();

    time_elapsed = duration_cast<microseconds>(high_resolution_clock::now() - start_time);

    temporal::Duration simulation_time_covered = scene->get_max_temporal_limit() - simulation_start_time;
    float real_time_factor = float(duration_cast<microseconds>(simulation_time_covered).count()) / float(time_elapsed.count());

    std::cout << "Finished simulation (" << time_elapsed.count() << " μs, rtf = " << real_time_factor << ")" << std::endl;

    delete map_simulated_intervened_scene_widget;
    delete map_simulated_scene_widget;
    delete map_scene_widget;

    delete frame;

    delete simulated_intervened_scene;
    delete simulated_scene;
    delete intervened_scene;
    delete scene_with_actions;
    delete scene;

    delete driving_simulator;
    delete driving_agent_controller;

    delete map;

    geometry::TrigBuff::destroy_instance();

    return result;
}
