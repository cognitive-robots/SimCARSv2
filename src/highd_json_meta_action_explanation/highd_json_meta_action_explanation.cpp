
#include <ori/simcars/structures/stl/stl_set.hpp>
#include <ori/simcars/geometry/trig_buff.hpp>
#include <ori/simcars/map/highd/highd_map.hpp>
#include <ori/simcars/causal/variable_context.hpp>
#include <ori/simcars/agents/fwd_car_action.hpp>
#include <ori/simcars/agents/fwd_car_action_extractor.hpp>
#include <ori/simcars/agents/fwd_car_sim.hpp>
#include <ori/simcars/agents/full_control_fwd_car_sim.hpp>
#include <ori/simcars/agents/action_intervention_fwd_car.hpp>
#include <ori/simcars/agents/default_fwd_car_outcome_sim.hpp>
#include <ori/simcars/agents/default_fwd_car_reward_calc.hpp>
#include <ori/simcars/agents/greedy_plan_fwd_car.hpp>
#include <ori/simcars/agents/highd/highd_fwd_car_scene.hpp>
#include <ori/simcars/visualisation/qdriving_map_agents_widget.hpp>

#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/ostreamwrapper.h>
#include <rapidjson/writer.h>

#include <iostream>
#include <filesystem>
#include <exception>
#include <chrono>

#define GOLDEN_RATIO_MAGIC_NUM 0x9e3779b9

using namespace ori::simcars;

template <typename T1, typename T2>
class PairHasher
{
    std::hash<T1> hasher_1;
    std::hash<T2> hasher_2;

public:
    std::size_t operator()(std::pair<T1, T2> const &key) const
    {
        size_t key_hash = hasher_1(key.first);
        key_hash ^= hasher_2(key.second) + GOLDEN_RATIO_MAGIC_NUM + (key_hash << 6) + (key_hash >> 2);
        return key_hash;
    }
};

int main(int argc, char *argv[])
{
    if (argc < 4)
    {
        std::cerr << "Usage: ./highd_json_meta_action_explanation intervention_impact_threshold "
                     "input_json_meta_file_path trimmed_data_directory_path "
                     "[output_json_meta_file_path] [restrict_to_relevant_agents]" <<
                     std::endl;
        return -1;
    }

    FP_DATA_TYPE intervention_impact_threshold = std::atof(argv[1]);

    std::cout << "Intervention Impact Threshold: " <<
                 std::to_string(intervention_impact_threshold) << std::endl;

    bool restrict_to_relevant_agents;
    if (argc > 5)
    {
        restrict_to_relevant_agents = std::atoi(argv[5]) == 0 ? false : true;
    }
    else
    {
        restrict_to_relevant_agents = false;
    }


    std::string input_json_meta_file_path_str = argv[2];
    std::filesystem::path input_json_meta_file_path(input_json_meta_file_path_str);

    if (!std::filesystem::is_regular_file(input_json_meta_file_path))
    {
        throw std::invalid_argument("Input JSON meta file path '" +
                                    input_json_meta_file_path_str +
                                    "' does not indicate a valid file");
    }

    std::string output_json_meta_file_path_str;
    std::filesystem::path output_json_meta_file_path;

    if (argc > 4)
    {
        output_json_meta_file_path_str = argv[4];
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
    rapidjson::Value const &independent_id_arr = json_meta_document["independent_ids"];
    structures::stl::STLStackArray<uint32_t> independent_ids;
    for (rapidjson::SizeType i = 0; i < independent_id_arr.Size(); ++i)
    {
        uint32_t independent_id = independent_id_arr[i].GetUint();
        independent_ids.push_back(independent_id);
    }

    std::string raw_data_directory_path_str = argv[3];
    std::filesystem::path raw_data_directory_path(raw_data_directory_path_str);

    std::filesystem::path recording_meta_file_path =
            raw_data_directory_path /
            ("scene-" + std::to_string(scene_id) + "-" + std::to_string(convoy_tail_id) +
             "_follows_" + std::to_string(convoy_head_id) + "-" +
             std::to_string(independent_ids.count()) + "_independent-recordingMeta.csv");

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
             std::to_string(independent_ids.count()) + "_independent-tracksMeta.csv");

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
             std::to_string(independent_ids.count()) + "_independent-tracks.csv");

    if (!std::filesystem::is_regular_file(tracks_file_path))
    {
        throw std::invalid_argument("Tracks file path '" +
                                    tracks_file_path.string() +
                                    "' does not indicate a valid file");
    }


    geometry::TrigBuff::init_instance(360000, geometry::AngleType::RADIANS);


    std::cout << "Beginning map load" << std::endl;

    map::highd::HighDMap map;

    map.load(recording_meta_file_path.string());

    std::cout << "Finished map load" << std::endl;


    std::cout << "Beginning scene load" << std::endl;

    agents::highd::HighDFWDCarScene scene(tracks_meta_file_path.string(),
                                          tracks_file_path.string());

    causal::VariableContext::set_time_step_size(scene.get_time_step_size());
    //causal::VariableContext::set_time_step_size(temporal::Duration(10));

    structures::IArray<agents::FWDCar*> const *fwd_cars = scene.get_fwd_cars();

    std::cout << "Finished scene load" << std::endl;


    std::chrono::time_point<std::chrono::high_resolution_clock> start_time =
            std::chrono::high_resolution_clock::now();


    agents::FWDCarActionExtractor fwd_car_action_extractor(&map, temporal::Duration(480),
                                                           temporal::Duration(2480), 0.1,
                                                           1.0, temporal::Duration(480));

    structures::stl::STLDictionary<uint64_t, agents::FWDCar*> id_fwd_car_dict;
    structures::stl::STLDictionary<uint64_t, structures::IArray<agents::TimeFWDCarActionPair>*> id_action_dict;

    size_t u;
    for (u = 0; u < fwd_cars->count(); ++u)
    {
        agents::FWDCar *fwd_car = (*fwd_cars)[u];

        uint64_t id;
        bool res = fwd_car->get_id_variable()->get_value(id);

        if (!res)
        {
            throw std::runtime_error("Could not get FWD car id");
        }

        //std::cout << "Extracting actions for agent " << id << std::endl;

        id_fwd_car_dict.update(id, fwd_car);

        structures::IArray<agents::TimeFWDCarActionPair> *fwd_car_actions =
                fwd_car_action_extractor.extract_actions(fwd_car);

        id_action_dict.update(id, fwd_car_actions);
    }


    structures::IArray<uint64_t> const *ids = id_action_dict.get_keys();
    // TODO: Use a more elegant means of capturing the causal links between actions
    structures::stl::STLDictionary<std::pair<uint64_t, uint64_t>, temporal::Time, PairHasher<uint64_t, uint64_t>> causal_links;
    size_t i, j, k, l;
    for (u = 0; u < ids->count(); ++u)
    {
        uint64_t affected_agent_id = (*ids)[u];
        agents::FWDCar *affected_fwd_car = id_fwd_car_dict[affected_agent_id];
        structures::IArray<agents::TimeFWDCarActionPair> *affected_time_action_pairs =
                id_action_dict[affected_agent_id];

        if (restrict_to_relevant_agents && affected_agent_id != convoy_head_id &&
                affected_agent_id != convoy_tail_id && !independent_ids.contains(affected_agent_id))
        {
            continue;
        }

        agents::FWDCarAction default_fwd_car_action;
        agents::RectRigidBodyEnv *original_env = scene.get_env();
        temporal::Time end_time = scene.get_max_time();
        for (i = 1; i < affected_time_action_pairs->count(); ++i)
        {
            std::cout << "Processing explanation for action " << i << " of agent " <<
                         affected_agent_id << std::endl;

            agents::TimeFWDCarActionPair affected_time_action_pair = (*affected_time_action_pairs)[i];

            agents::FWDCarOutcomeActionPair affected_outcome_action_pair;
            temporal::Duration sim_horizon(0);
            if (i == affected_time_action_pairs->count() - 1)
            {
                causal::IEndogenousVariable<FP_DATA_TYPE> *lon_lin_vel =
                        affected_fwd_car->get_lon_lin_vel_variable();
                causal::VariableContext::set_current_time(affected_time_action_pair.first);
                lon_lin_vel->get_value(affected_outcome_action_pair.first.final_speed);
                while (true)
                {
                    causal::VariableContext::set_current_time(affected_time_action_pair.first +
                                                              sim_horizon +
                                                              causal::VariableContext::get_time_step_size());
                    FP_DATA_TYPE speed;
                    if (lon_lin_vel->get_value(speed))
                    {
                        sim_horizon += causal::VariableContext::get_time_step_size();
                        affected_outcome_action_pair.first.final_speed = speed;
                    }
                    else
                    {
                        break;
                    }
                }
            }
            else
            {
                sim_horizon = (*affected_time_action_pairs)[i + 1].first -
                        affected_time_action_pair.first;
                causal::VariableContext::set_current_time(affected_time_action_pair.first + sim_horizon);
                affected_fwd_car->get_lon_lin_vel_variable()->get_value(
                            affected_outcome_action_pair.first.final_speed);
            }
            affected_fwd_car->get_dist_headway_variable()->get_value(
                        affected_outcome_action_pair.first.dist_headway);
            causal::LaneEncapsulatingVariable lane_encaps(affected_fwd_car->get_pos_variable(), &map);
            causal::IdsPreviousTimeStepVariable prev_lane_encaps(&lane_encaps);
            causal::LaneTransitionsCalcVariable lane_trans(&prev_lane_encaps, &lane_encaps, &map);
            affected_outcome_action_pair.first.lane_transitions = 0.0;
            temporal::Time current_time;
            for (current_time = affected_time_action_pair.first +
                 causal::VariableContext::get_time_step_size();
                 current_time <= affected_time_action_pair.first + sim_horizon;
                 current_time += causal::VariableContext::get_time_step_size())
            {
                causal::VariableContext::set_current_time(current_time);
                FP_DATA_TYPE lane_transitions;
                lane_trans.get_value(lane_transitions);
                affected_outcome_action_pair.first.lane_transitions += lane_transitions;
            }
            for (current_time = affected_time_action_pair.first;
                 current_time <= affected_time_action_pair.first + sim_horizon;
                 current_time += causal::VariableContext::get_time_step_size())
            {
                causal::VariableContext::set_current_time(current_time);
                geometry::Vec env_force;
                affected_fwd_car->get_env_force_variable()->get_value(env_force);
                if (current_time == affected_time_action_pair.first)
                {
                    affected_outcome_action_pair.first.max_env_force_mag = env_force.norm();
                }
                else
                {
                    affected_outcome_action_pair.first.max_env_force_mag = std::max(
                                env_force.norm(),
                                affected_outcome_action_pair.first.max_env_force_mag);
                }
            }
            affected_outcome_action_pair.first.action_done = true;
            affected_outcome_action_pair.second = affected_time_action_pair.second;

            // TODO: Integrate better information regarding braking
            agents::FullControlFWDCar affected_control_fwd_car(&map, 163 * 20, -163 * 20, 0.616);
            agents::DefaultFWDCarOutcomeSim original_outcome_sim(&affected_control_fwd_car,
                                                                 original_env);
            agents::FWDCarSimParameters outcome_sim_params = {
                .sim_horizon_secs = std::chrono::duration_cast<std::chrono::duration<FP_DATA_TYPE>>(
                sim_horizon).count(),
                .action_done_final_speed_threshold = 1.0
            };
            agents::DefaultFWDCarRewardCalc reward_calc;
            agents::FWDCarRewardParameters reward_calc_params = {
                .speed_limit = 31.3,
                .env_force_mag_limit = 1000
            };
            agents::GreedyPlanFWDCar affected_original_plan_fwd_car(&map, &original_outcome_sim,
                                                                    outcome_sim_params, &reward_calc,
                                                                    reward_calc_params, 0, 45, 2.5, 5, 2.5);
            affected_control_fwd_car.set_fwd_car(affected_fwd_car);
            affected_original_plan_fwd_car.set_control_fwd_car(&affected_control_fwd_car);

            causal::VariableContext::set_current_time(affected_time_action_pair.first);
            causal::IEndogenousVariable<agents::FWDCarOutcomeActionPair> *affected_original_fwd_car_best_action =
                    affected_original_plan_fwd_car.get_best_outcome_action_pair_variable();
            affected_original_fwd_car_best_action->set_value(affected_outcome_action_pair);
            agents::FWDCarOutcomeActionPair affected_best_sim_outcome_action_pair;
            affected_original_fwd_car_best_action->get_value(affected_best_sim_outcome_action_pair);
            agents::FWDCarAction affected_best_sim_action = affected_best_sim_outcome_action_pair.second;
            causal::IEndogenousVariable<agents::FWDCarRewardParameters> *affected_original_fwd_car_reward_params =
                    affected_original_plan_fwd_car.get_reward_params_variable();
            affected_original_fwd_car_reward_params->get_value(reward_calc_params);

            affected_control_fwd_car.set_fwd_car(nullptr);

            std::cout << "Reward Parameters: { lane_transition: " <<
                         reward_calc_params.lane_transitions_weight << ", dist_headway: " <<
                         reward_calc_params.dist_headway_weight << ", max_speed: " <<
                         reward_calc_params.max_speed_weight << ", anti_speed: " <<
                         reward_calc_params.anti_speed_weight << ", max_env_force_mag: " <<
                         reward_calc_params.max_env_force_mag_weight << ", bias: " <<
                         reward_calc_params.bias_weight << " }" << std::endl;

            for (j = 0; j < ids->count(); ++j)
            {
                uint64_t causing_agent_id = (*ids)[j];
                agents::FWDCar *causing_fwd_car = id_fwd_car_dict[causing_agent_id];

                if (restrict_to_relevant_agents && causing_agent_id != convoy_head_id &&
                        causing_agent_id != convoy_tail_id && !independent_ids.contains(causing_agent_id))
                {
                    continue;
                }

                if (causing_agent_id == affected_agent_id)
                {
                    continue;
                }
                else
                {
                    structures::IArray<agents::TimeFWDCarActionPair> *causing_time_action_pairs =
                            id_action_dict[causing_agent_id];

                    for (k = 1; k < causing_time_action_pairs->count(); ++k)
                    {
                        agents::TimeFWDCarActionPair causing_time_action_pair = (*causing_time_action_pairs)[k];

                        if (causing_time_action_pair.first >= affected_time_action_pair.first)
                        {
                            continue;
                        }

                        if (k + 1 < causing_time_action_pairs->count())
                        {
                            agents::TimeFWDCarActionPair next_causing_time_action_pair =
                                    (*causing_time_action_pairs)[k + 1];
                            if (next_causing_time_action_pair.first < affected_time_action_pair.first)
                            {
                                continue;
                            }
                        }

                        //std::cout << "Testing link: Action " << k << " of agent " << causing_agent_id <<
                        //             " -> Action " << i << " of agent " << affected_agent_id <<
                        //             std::endl;

                        //agents::RectRigidBodyEnv env;

                        /*
                    for (l = 0; l < ids->count(); ++l)
                    {
                        uint64_t id = (*ids)[l];
                        if (id != affected_agent_id && l != j)
                        {
                            env.add_rigid_body(id_fwd_car_dict[id]);
                        }
                    }
                    */


                        agents::FWDCarSim causing_fwd_car_sim(causing_fwd_car, causing_time_action_pair.first);

                        // TODO: Integrate better information regarding braking
                        agents::FullControlFWDCar causing_control_fwd_car(&map, 163 * 20, -163, 0.616);
                        agents::FullControlFWDCarSim causing_control_fwd_car_sim(
                                    &causing_control_fwd_car, causing_time_action_pair.first -
                                    causal::VariableContext::get_time_step_size());

                        agents::ActionInterventionFWDCar causing_plan_fwd_car(default_fwd_car_action);

                        causal::IEndogenousVariable<agents::FWDCarAction> *causing_action_intervention_fwd_car =
                                causing_plan_fwd_car.get_action_intervention_variable();
                        for (l = 0; l < k; ++l)
                        {
                            agents::TimeFWDCarActionPair current_action = (*causing_time_action_pairs)[l];
                            for (temporal::Time current_time = current_action.first;
                                 current_time <= end_time;
                                 current_time += causal::VariableContext::get_time_step_size())
                            {
                                causal::VariableContext::set_current_time(current_time);
                                causing_action_intervention_fwd_car->set_value(current_action.second);
                            }
                        }

                        causing_control_fwd_car_sim.set_fwd_car(&causing_fwd_car_sim);
                        causing_plan_fwd_car.set_control_fwd_car(&causing_control_fwd_car_sim);

                        //env.add_rigid_body(&causing_fwd_car_sim);
                        original_env->remove_rigid_body(causing_fwd_car);
                        original_env->add_rigid_body(&causing_fwd_car_sim);


                        agents::FWDCarSim affected_fwd_car_sim(affected_fwd_car, affected_time_action_pair.first);

                        agents::FullControlFWDCarSim affected_control_fwd_car_sim(
                                    &affected_control_fwd_car, affected_time_action_pair.first -
                                    causal::VariableContext::get_time_step_size());

                        agents::DefaultFWDCarOutcomeSim outcome_sim(&affected_control_fwd_car_sim,
                                                                    original_env);
                        agents::GreedyPlanFWDCar affected_plan_fwd_car(&map, &outcome_sim,
                                                                       outcome_sim_params, &reward_calc,
                                                                       reward_calc_params, 0, 45, 2.5,
                                                                       5, 2.5);

                        affected_control_fwd_car_sim.set_fwd_car(&affected_fwd_car_sim);
                        affected_plan_fwd_car.set_control_fwd_car(&affected_control_fwd_car_sim);

                        //env.add_rigid_body(&affected_fwd_car_sim);
                        original_env->remove_rigid_body(affected_fwd_car);
                        original_env->add_rigid_body(&affected_fwd_car_sim);


                        causal::VariableContext::set_current_time(affected_time_action_pair.first);
                        causal::IEndogenousVariable<agents::FWDCarOutcomeActionPair> *affected_fwd_car_best_action =
                                affected_plan_fwd_car.get_best_outcome_action_pair_variable();
                        agents::FWDCarOutcomeActionPair affected_best_alt_sim_outcome_action_pair;
                        affected_fwd_car_best_action->get_value(affected_best_alt_sim_outcome_action_pair);
                        agents::FWDCarAction affected_best_alt_sim_action =
                                affected_best_alt_sim_outcome_action_pair.second;


                        original_env->remove_rigid_body(&affected_fwd_car_sim);
                        original_env->add_rigid_body(affected_fwd_car);

                        original_env->remove_rigid_body(&causing_fwd_car_sim);
                        original_env->add_rigid_body(causing_fwd_car);

                        // Compare affected_time_action_pair, affected_best_sim_action and affected_best_alt_sim_action

                        /*
                    std::cout << "Action Time: " <<
                                 std::to_string(affected_time_action_pair.first.time_since_epoch().count() /
                                                1000) << " s" << std::endl;
                    std::cout << "affected Action: " << affected_time_action_pair.second << std::endl;
                    std::cout << "affected Best Sim. Action: " << affected_best_sim_action << std::endl;
                    std::cout << "affected Best Alt. Sim. Action: " << affected_best_alt_sim_action << std::endl;
                    */

                        FP_DATA_TYPE representativeness = diff(affected_time_action_pair.second,
                                                               affected_best_sim_action);
                        FP_DATA_TYPE alt_representativeness = diff(affected_time_action_pair.second,
                                                                   affected_best_sim_action);
                        FP_DATA_TYPE intervention_impact = diff(affected_best_sim_action,
                                                                affected_best_alt_sim_action);

                        //std::cout << "Intervention Impact: " << intervention_impact << std::endl;

                        if (//representativeness < alt_representativeness &&
                                //representativeness < intervention_impact &&
                                intervention_impact > intervention_impact_threshold)
                        {
                            std::cout << "Action " << k << " of agent " << causing_agent_id <<
                                         " -> Action " << i << " of agent " << affected_agent_id <<
                                         std::endl;
                            std::cout << "Action Time: " <<
                                         std::to_string(affected_time_action_pair.first.time_since_epoch().count() /
                                                        1000) << " s" << std::endl;
                            std::cout << "affected Action: " << affected_time_action_pair.second << std::endl;
                            std::cout << "affected Outcome: " << affected_outcome_action_pair.first << std::endl;
                            std::cout << "affected Best Sim. Action: " << affected_best_sim_action << std::endl;
                            std::cout << "affected Best Sim. Outcome: " << affected_best_sim_outcome_action_pair.first << std::endl;
                            std::cout << "affected Best Alt. Sim. Action: " << affected_best_alt_sim_action << std::endl;
                            std::cout << "affected Best Alt. Sim. Outcome: " << affected_best_alt_sim_outcome_action_pair.first << std::endl;

                            std::pair<uint64_t, uint64_t> agent_id_pair(causing_agent_id, affected_agent_id);
                            /*
                            if (!causal_links.contains(agent_id_pair) ||
                                    causal_links[agent_id_pair] > causing_time_action_pair.first)
                            {
                                causal_links.update(agent_id_pair, causing_time_action_pair.first);
                            }
                            */
                            if (!causal_links.contains(agent_id_pair))
                            {
                                causal_links.update(agent_id_pair, causing_time_action_pair.first);
                            }
                        }
                    }
                }
            }
        }
    }


    std::chrono::microseconds time_elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::high_resolution_clock::now() - start_time);


    geometry::TrigBuff::destroy_instance();


    if (argc > 4)
    {
        structures::IArray<std::pair<uint64_t, uint64_t>> const *causal_link_array =
                causal_links.get_keys();

        rapidjson::Value json_causal_links(rapidjson::kObjectType);
        for (i = 0; i < causal_link_array->count(); ++i)
        {
            std::pair<uint64_t, uint64_t> const &causal_link = (*causal_link_array)[i];
            //std::pair<uint64_t, uint64_t> const reverse_causal_link(causal_link.second, causal_link.first);

            /*
            if (causal_links.contains(reverse_causal_link) &&
                    causal_links[reverse_causal_link] < causal_links[causal_link])
            {
                continue;
            }
            */

            std::string cause_str = std::to_string(causal_link.first);
            rapidjson::Value cause_json_str;
            cause_json_str.SetString(cause_str.c_str(), json_meta_document.GetAllocator());
            rapidjson::Value effect_json_uint;
            effect_json_uint.SetUint(causal_link.second);

            if (!json_causal_links.HasMember(cause_json_str))
            {
                rapidjson::Value json_causal_effects(rapidjson::kArrayType);
                json_causal_effects.PushBack(effect_json_uint, json_meta_document.GetAllocator());
                json_causal_links.AddMember(cause_json_str, json_causal_effects,
                                                   json_meta_document.GetAllocator());
            }
            else
            {
                rapidjson::Value::Array const &json_causal_effects =
                        json_causal_links[cause_json_str].GetArray();
                json_causal_effects.PushBack(effect_json_uint, json_meta_document.GetAllocator());
            }
        }
        json_meta_document.AddMember("causal_links", json_causal_links,
                                     json_meta_document.GetAllocator());

        rapidjson::Value time_elapsed_in_microseconds;
        time_elapsed_in_microseconds.SetInt64(time_elapsed.count());
        json_meta_document.AddMember("time_elapsed_in_microseconds", time_elapsed_in_microseconds,
                                     json_meta_document.GetAllocator());

        std::ofstream output_json_meta_filestream(output_json_meta_file_path);
        rapidjson::OStreamWrapper output_json_meta_stream(output_json_meta_filestream);
        rapidjson::Writer<rapidjson::OStreamWrapper> output_json_meta_writer(
                    output_json_meta_stream);
        json_meta_document.Accept(output_json_meta_writer);
    }

}
