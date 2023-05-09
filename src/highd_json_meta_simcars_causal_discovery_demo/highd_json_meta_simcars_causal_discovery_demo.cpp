
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
#include <ori/simcars/causal/necessary_driving_causal_discoverer.hpp>

#include <QApplication>
#include <QFrame>
#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>

#include <iostream>
#include <exception>
#include <random>
#include <string>
#include <fstream>
#include <filesystem>

using namespace ori::simcars;

int main(int argc, char *argv[])
{
    if (argc < 3)
    {
        std::cerr << "Usage: ./highd_json_meta_simcars_causal_discovery_demo json_meta_file_path "
                     "trimmed_data_directory_path [potential_causal_link_index]" << std::endl;
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

    std::string convoy_head_str = "non_ego_vehicle_" + std::to_string(convoy_head_id);
    std::string convoy_tail_str = "non_ego_vehicle_" + std::to_string(convoy_tail_id);
    std::string independent_str = "non_ego_vehicle_" + std::to_string(independent_id);

    std::cout << "Convoy Head Agent = " << convoy_head_str << std::endl;
    std::cout << "Convoy Tail Agent = " << convoy_tail_str << std::endl;
    std::cout << "Independent Agent = " << independent_str << std::endl;

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

    std::cout << "Beginning map load" << std::endl;

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

    std::cout << "Finished map load" << std::endl;


    std::cout << "Beginning scene load" << std::endl;

    agent::IDrivingScene *driving_scene;

    try
    {
        driving_scene = agent::highd::HighDScene::load(tracks_meta_file_path.string(),
                                               tracks_file_path.string());
    }
    catch (std::exception const &e)
    {
        std::cerr << "Exception occured during scene load:" << std::endl << e.what() << std::endl;
        return -1;
    }

    std::cout << "Finished scene load" << std::endl;


    std::cout << "Beginning action extraction" << std::endl;

    agent::IDrivingScene *driving_scene_with_actions;

    try
    {
        driving_scene_with_actions =
                agent::DrivingGoalExtractionScene<uint8_t>::construct_from(driving_scene, map);
    }
    catch (std::exception const &e)
    {
        std::cerr << "Exception occured during action extraction:" << std::endl << e.what() << std::endl;
        return -1;
    }

    std::cout << "Finished action extraction" << std::endl;


    structures::IArray<agent::IDrivingAgent*> const *driving_agents_with_actions =
            driving_scene_with_actions->get_mutable_driving_agents();

    structures::stl::STLStackArray<agent::IEvent<agent::Goal<FP_DATA_TYPE>> const*> aligned_linear_velocity_goal_events;

    size_t i, j;
    for (i = 0; i < driving_agents_with_actions->count(); ++i)
    {
        agent::IDrivingAgent *driving_agent_with_actions =
                (*driving_agents_with_actions)[i];
        if (agents_of_interest == nullptr ||
                agents_of_interest->contains(driving_agent_with_actions->get_name()))
        {
            agent::IValuelessVariable *driving_agent_aligned_linear_velocity_goal_valueless_variable =
                    driving_agent_with_actions->get_mutable_variable_parameter(
                        driving_agent_with_actions->get_name() + ".aligned_linear_velocity.goal");

            if (driving_agent_aligned_linear_velocity_goal_valueless_variable == nullptr)
            {
                throw std::runtime_error("No aligned linear velocity goal variable present on agent of interest");
            }

            agent::IVariable<agent::Goal<FP_DATA_TYPE>> *driving_agent_aligned_linear_velocity_goal_variable =
                    dynamic_cast<agent::IVariable<agent::Goal<FP_DATA_TYPE>>*>(
                            driving_agent_aligned_linear_velocity_goal_valueless_variable);

            structures::IArray<agent::IEvent<agent::Goal<FP_DATA_TYPE>> const*> const *driving_agent_aligned_linear_velocity_goal_events =
                    driving_agent_aligned_linear_velocity_goal_variable->get_events();

            for (j = 0; j < driving_agent_aligned_linear_velocity_goal_events->count(); ++j)
            {
                agent::IEvent<agent::Goal<FP_DATA_TYPE>> const *driving_agent_aligned_linear_velocity_goal_event =
                        (*driving_agent_aligned_linear_velocity_goal_events)[j];

                aligned_linear_velocity_goal_events.push_back(
                            driving_agent_aligned_linear_velocity_goal_event);
            }

            delete driving_agent_aligned_linear_velocity_goal_events;

            agent::IVariable<FP_DATA_TYPE> *driving_agent_aligned_linear_velocity_variable =
                    driving_agent_with_actions->get_mutable_aligned_linear_velocity_variable();

            FP_DATA_TYPE driving_agent_initial_aligned_linear_velocity;

            if (!driving_agent_aligned_linear_velocity_variable->get_value(
                        driving_agent_with_actions->get_min_temporal_limit(),
                        driving_agent_initial_aligned_linear_velocity))
            {
                throw std::runtime_error("Could not get initial aligned linear velocity of "
                                         "driving agent");
            }

            temporal::Time just_before_start =
                    driving_agent_with_actions->get_min_temporal_limit() -
                    driving_scene_with_actions->get_time_step();

            driving_agent_aligned_linear_velocity_goal_variable->set_value(
                        just_before_start,
                        agent::Goal(driving_agent_initial_aligned_linear_velocity,
                                    just_before_start));
        }
    }

    delete driving_agents_with_actions;

    delete agents_of_interest;

    int result;
    if (argc == 3)
    {
        size_t k = 0;
        for (i = 0; i < aligned_linear_velocity_goal_events.count(); ++i)
        {
            agent::IEvent<agent::Goal<FP_DATA_TYPE>> const *potential_cause =
                    aligned_linear_velocity_goal_events[i];
            for (j = 0; j < aligned_linear_velocity_goal_events.count(); ++j)
            {
                agent::IEvent<agent::Goal<FP_DATA_TYPE>> const *potential_effect =
                        aligned_linear_velocity_goal_events[j];

                if (potential_cause->get_entity_name() != potential_effect->get_entity_name() &&
                        potential_cause->get_time() < potential_effect->get_time())
                {
                    agent::IDrivingAgent const *cause_agent =
                            driving_scene_with_actions->get_driving_agent(
                                potential_cause->get_entity_name());
                    agent::IDrivingAgent const *effect_agent =
                            driving_scene_with_actions->get_driving_agent(
                                potential_effect->get_entity_name());

                    agent::IVariable<FP_DATA_TYPE> const *cause_speed_variable =
                            cause_agent->get_aligned_linear_velocity_variable();
                    agent::IVariable<FP_DATA_TYPE> const *effect_speed_variable =
                            effect_agent->get_aligned_linear_velocity_variable();

                    FP_DATA_TYPE cause_speed;
                    FP_DATA_TYPE effect_speed;

                    cause_speed_variable->get_value(potential_cause->get_time(), cause_speed);
                    effect_speed_variable->get_value(potential_effect->get_time(), effect_speed);

                    std::cout << "[" << k << "] " << cause_agent->get_name() << " (" <<
                                 cause_speed * 1e3f << " m/s -> " <<
                                 potential_cause->get_value().get_goal_value() * 1e3f <<
                                 " m/s) -> " << effect_agent->get_name() << " (" <<
                                 effect_speed * 1e3f << " m/s -> " <<
                                 potential_effect->get_value().get_goal_value() * 1e3f <<
                                 " m/s)" << std::endl;

                    ++k;
                }
            }
        }

        result = 0;
    }
    else
    {
        size_t potential_causal_link_index = std::atoi(argv[3]);

        agent::IEvent<agent::Goal<FP_DATA_TYPE>> const *selected_potential_cause = nullptr;
        agent::IEvent<agent::Goal<FP_DATA_TYPE>> const *selected_potential_effect = nullptr;

        size_t k = 0;
        for (i = 0; i < aligned_linear_velocity_goal_events.count(); ++i)
        {
            agent::IEvent<agent::Goal<FP_DATA_TYPE>> const *potential_cause =
                    aligned_linear_velocity_goal_events[i];
            for (j = 0; j < aligned_linear_velocity_goal_events.count(); ++j)
            {
                agent::IEvent<agent::Goal<FP_DATA_TYPE>> const *potential_effect =
                        aligned_linear_velocity_goal_events[j];

                if (potential_cause->get_entity_name() != potential_effect->get_entity_name() &&
                        potential_cause->get_time() < potential_effect->get_time())
                {
                    if (k == potential_causal_link_index)
                    {
                        selected_potential_cause = potential_cause;
                        selected_potential_effect = potential_effect;
                        break;
                    }
                    else
                    {
                        ++k;
                    }
                }
            }
            if (selected_potential_cause != nullptr && selected_potential_effect != nullptr)
            {
                break;
            }
        }
        if (selected_potential_cause == nullptr || selected_potential_effect == nullptr)
        {
            throw std::invalid_argument("Specified potential causal link not found");
        }

        agent::IDrivingScene *original_scene = driving_scene_with_actions;

        temporal::Time time_window_start = selected_potential_cause->get_time();
        temporal::Time time_window_end = original_scene->get_max_temporal_limit();

        agent::IEntity *cause_entity = original_scene->get_mutable_entity(
                    selected_potential_cause->get_entity_name());
        agent::IValuelessVariable *cause_variable =
                cause_entity->get_mutable_variable_parameter(
                    selected_potential_cause->get_full_name());
        cause_variable->propogate_events_forward(original_scene->get_max_temporal_limit());

        agent::IEntity *effect_entity = original_scene->get_mutable_entity(
                    selected_potential_effect->get_entity_name());
        agent::IValuelessVariable *effect_variable =
                effect_entity->get_mutable_variable_parameter(
                    selected_potential_effect->get_full_name());
        effect_variable->propogate_events_forward(original_scene->get_max_temporal_limit());


        structures::ISet<std::string> *relevant_agent_names = new structures::stl::STLSet<std::string>;
        relevant_agent_names->insert(selected_potential_cause->get_entity_name());
        relevant_agent_names->insert(selected_potential_effect->get_entity_name());


        agent::IDrivingAgentController *driving_agent_controller =
                    new agent::BasicDrivingAgentController<uint8_t>(map,
                                                                    original_scene->get_time_step(),
                                                                    10);

        agent::IDrivingSimulator *driving_simulator =
                    new agent::BasicDrivingSimulator(driving_agent_controller);


        agent::IDrivingScene *cause_intervened_scene = original_scene->driving_scene_deep_copy();
        agent::IEntity *cause_intervened_entity =
                cause_intervened_scene->get_mutable_entity(
                    selected_potential_cause->get_entity_name());
        agent::IValuelessVariable *cause_intervened_variable =
                cause_intervened_entity->get_mutable_variable_parameter(
                    selected_potential_cause->get_full_name());
        cause_intervened_variable->remove_value(selected_potential_cause->get_time());
        cause_intervened_variable->propogate_events_forward(
                    original_scene->get_max_temporal_limit());
        agent::IDrivingScene *simulated_cause_intervened_scene =
                agent::DrivingSimulationScene::construct_from(
                    cause_intervened_scene, driving_simulator,
                    cause_intervened_scene->get_time_step(), time_window_start, time_window_end,
                    relevant_agent_names);

        agent::IDrivingScene *effect_intervened_scene = original_scene->driving_scene_deep_copy();
        agent::IEntity *effect_intervened_entity =
                effect_intervened_scene->get_mutable_entity(
                    selected_potential_effect->get_entity_name());
        agent::IValuelessVariable *effect_intervened_variable =
                effect_intervened_entity->get_mutable_variable_parameter(
                    selected_potential_effect->get_full_name());
        effect_intervened_variable->remove_value(
                    selected_potential_effect->get_time());
        effect_intervened_variable->propogate_events_forward(
                    original_scene->get_max_temporal_limit());
        agent::IDrivingScene *simulated_effect_intervened_scene =
                agent::DrivingSimulationScene::construct_from(
                    effect_intervened_scene, driving_simulator,
                    effect_intervened_scene->get_time_step(), time_window_start, time_window_end,
                    relevant_agent_names);

        agent::IDrivingScene *cause_effect_intervened_scene =
                cause_intervened_scene->driving_scene_deep_copy();
        agent::IEntity *cause_effect_intervened_entity =
                cause_effect_intervened_scene->get_mutable_entity(
                    selected_potential_effect->get_entity_name());
        agent::IValuelessVariable *cause_effect_intervened_variable =
                cause_effect_intervened_entity->get_mutable_variable_parameter(
                    selected_potential_effect->get_full_name());
        cause_effect_intervened_variable->remove_value(
                    selected_potential_effect->get_time());
        cause_effect_intervened_variable->propogate_events_forward(
                    original_scene->get_max_temporal_limit());
        agent::IDrivingScene *simulated_cause_effect_intervened_scene =
                agent::DrivingSimulationScene::construct_from(
                    cause_effect_intervened_scene, driving_simulator,
                    cause_effect_intervened_scene->get_time_step(), time_window_start,
                    time_window_end, relevant_agent_names);


        std::cout << "Beginning simulation" << std::endl;

        QFrame *frame = new QFrame();
        frame->setWindowTitle("SIMCARS Causal Discovery Demo");
        frame->setFixedSize(1200, 1200);
        frame->show();

        visualisation::QMapSceneWidget<uint8_t> *map_original_scene_widget =
                    new visualisation::QMapSceneWidget<uint8_t>(
                        map,
                        original_scene,
                        frame,
                        QPoint(20, 20),
                        QSize(560, 560),
                        1.0f,
                        30.0f,
                        1.0f,
                        10.0f,
                        false);
        structures::IStackArray<std::string> *original_scene_focal_agents =
                new structures::stl::STLStackArray(relevant_agent_names->get_array());
        map_original_scene_widget->set_focal_entities(original_scene_focal_agents);
        map_original_scene_widget->set_focus_mode(
                    visualisation::QMapSceneWidget<uint8_t>::FocusMode::FOCAL_AGENTS);
        map_original_scene_widget->set_time(time_window_start);
        map_original_scene_widget->show();

        visualisation::QMapSceneWidget<uint8_t> *map_simulated_effect_intervened_scene_widget =
                    new visualisation::QMapSceneWidget<uint8_t>(
                        map,
                        simulated_effect_intervened_scene,
                        frame,
                        QPoint(620, 20),
                        QSize(560, 560),
                        1.0f,
                        30.0f,
                        1.0f,
                        10.0f,
                        false);
        structures::IStackArray<std::string> *simulated_effect_intervened_scene_focal_agents =
                new structures::stl::STLStackArray(relevant_agent_names->get_array());
        map_simulated_effect_intervened_scene_widget->set_focal_entities(
                    simulated_effect_intervened_scene_focal_agents);
        map_simulated_effect_intervened_scene_widget->set_focus_mode(
                    visualisation::QMapSceneWidget<uint8_t>::FocusMode::FOCAL_AGENTS);
        map_simulated_effect_intervened_scene_widget->set_time(time_window_start);
        map_simulated_effect_intervened_scene_widget->show();

        visualisation::QMapSceneWidget<uint8_t> *map_simulated_cause_intervened_scene_widget =
                    new visualisation::QMapSceneWidget<uint8_t>(
                        map,
                        simulated_cause_intervened_scene,
                        frame,
                        QPoint(20, 620),
                        QSize(560, 560),
                        1.0f,
                        30.0f,
                        1.0f,
                        10.0f,
                        false);
        structures::IStackArray<std::string> *simulated_cause_intervened_scene_focal_agents =
                new structures::stl::STLStackArray(relevant_agent_names->get_array());
        map_simulated_cause_intervened_scene_widget->set_focal_entities(
                    simulated_cause_intervened_scene_focal_agents);
        map_simulated_cause_intervened_scene_widget->set_focus_mode(
                    visualisation::QMapSceneWidget<uint8_t>::FocusMode::FOCAL_AGENTS);
        map_simulated_cause_intervened_scene_widget->set_time(time_window_start);
        map_simulated_cause_intervened_scene_widget->show();

        visualisation::QMapSceneWidget<uint8_t> *map_simulated_cause_effect_intervened_scene_widget =
                    new visualisation::QMapSceneWidget<uint8_t>(
                        map,
                        simulated_cause_effect_intervened_scene,
                        frame,
                        QPoint(620, 620),
                        QSize(560, 560),
                        1.0f,
                        30.0f,
                        1.0f,
                        10.0f,
                        false);
        structures::IStackArray<std::string> *simulated_cause_effect_intervened_scene_focal_agents =
                new structures::stl::STLStackArray(relevant_agent_names->get_array());
        map_simulated_cause_effect_intervened_scene_widget->set_focal_entities(
                    simulated_cause_effect_intervened_scene_focal_agents);
        map_simulated_cause_effect_intervened_scene_widget->set_focus_mode(
                    visualisation::QMapSceneWidget<uint8_t>::FocusMode::FOCAL_AGENTS);
        map_simulated_cause_effect_intervened_scene_widget->set_time(time_window_start);
        map_simulated_cause_effect_intervened_scene_widget->show();

        delete relevant_agent_names;

        result = app.exec();

        std::cout << "Finished simulation" << std::endl;

        delete map_simulated_cause_effect_intervened_scene_widget;
        delete map_simulated_cause_intervened_scene_widget;
        delete map_simulated_effect_intervened_scene_widget;
        delete map_original_scene_widget;

        delete frame;

        delete simulated_cause_effect_intervened_scene;
        delete cause_effect_intervened_scene;
        delete simulated_effect_intervened_scene;
        delete effect_intervened_scene;
        delete simulated_cause_intervened_scene;
        delete cause_intervened_scene;

        delete driving_simulator;
        delete driving_agent_controller;
    }

    delete driving_scene_with_actions;
    delete driving_scene;

    delete map;

    geometry::TrigBuff::destroy_instance();

    return result;
}
