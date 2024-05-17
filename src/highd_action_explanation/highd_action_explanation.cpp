
#include <ori/simcars/structures/stl/stl_set.hpp>
#include <ori/simcars/geometry/trig_buff.hpp>
#include <ori/simcars/map/highd/highd_map.hpp>
#include <ori/simcars/causal/variable_context.hpp>
#include <ori/simcars/agents/fwd_car_action.hpp>
#include <ori/simcars/agents/fwd_car_action_extractor.hpp>
#include <ori/simcars/agents/fwd_car_sim.hpp>
#include <ori/simcars/agents/full_control_fwd_car_sim.hpp>
#include <ori/simcars/agents/fwd_car_action_intervention.hpp>
#include <ori/simcars/agents/default_fwd_car_outcome_sim.hpp>
#include <ori/simcars/agents/default_fwd_car_reward_calc.hpp>
#include <ori/simcars/agents/greedy_plan_fwd_car.hpp>
#include <ori/simcars/agents/highd/highd_fwd_car_scene.hpp>
#include <ori/simcars/visualisation/qmap_agents_widget.hpp>

#include <iostream>
#include <exception>

using namespace ori::simcars;

int main(int argc, char *argv[])
{
    if (argc < 7)
    {
        std::cerr << "Usage: ./highd_action_explanation recording_meta_file_path tracks_meta_file_path "
                     "tracks_file_path start_frame end_frame agent_id" <<
                     std::endl;
        return -1;
    }

    size_t start_frame = atoi(argv[4]);
    size_t end_frame = atoi(argv[5]);

    geometry::TrigBuff::init_instance(360000, geometry::AngleType::RADIANS);


    std::cout << "Beginning map load" << std::endl;

    map::highd::HighDMap map;

    map.load(argv[1]);

    std::cout << "Finished map load" << std::endl;


    std::cout << "Beginning scene load" << std::endl;

    agents::highd::HighDFWDCarScene scene(argv[2], argv[3], start_frame, end_frame);

    causal::VariableContext::set_time_step_size(scene.get_time_step_size());
    //causal::VariableContext::set_time_step_size(temporal::Duration(10));

    structures::IArray<agents::FWDCar*> const *fwd_cars = scene.get_fwd_cars();

    std::cout << "Finished scene load" << std::endl;


    agents::FWDCarActionExtractor fwd_car_action_extractor(&map, temporal::Duration(500),
                                                           temporal::Duration(2500), 0.1,
                                                           1.0, temporal::Duration(500));

    structures::stl::STLDictionary<uint64_t, agents::FWDCar*> id_fwd_car_dict;
    structures::stl::STLDictionary<uint64_t, structures::IArray<agents::TimeFWDCarActionPair>*> id_action_dict;

    size_t i;
    for (i = 0; i < fwd_cars->count(); ++i)
    {
        agents::FWDCar *fwd_car = (*fwd_cars)[i];

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

    uint64_t effected_agent_id = atoi(argv[6]);
    agents::FWDCar *effected_fwd_car = id_fwd_car_dict[effected_agent_id];
    structures::IArray<agents::TimeFWDCarActionPair> *effected_time_action_pairs =
            id_action_dict[effected_agent_id];

    structures::IArray<uint64_t> const *ids = id_action_dict.get_keys();

    agents::FWDCarAction default_fwd_car_action;
    agents::RectRigidBodyEnv *original_env = scene.get_env();
    temporal::Time end_time = scene.get_max_time();
    size_t j, k, l;
    for (i = 1; i < effected_time_action_pairs->count(); ++i)
    {
        std::cout << "Processing explanation for action " << i << " of agent " <<
                     effected_agent_id << std::endl;

        agents::TimeFWDCarActionPair effected_time_action_pair = (*effected_time_action_pairs)[i];

        agents::FWDCarOutcomeActionPair effected_outcome_action_pair;
        temporal::Duration sim_horizon(0);
        if (i == effected_time_action_pairs->count() - 1)
        {
            causal::IEndogenousVariable<FP_DATA_TYPE> *lon_lin_vel =
                    effected_fwd_car->get_lon_lin_vel_variable();
            causal::VariableContext::set_current_time(effected_time_action_pair.first);
            lon_lin_vel->get_value(effected_outcome_action_pair.first.final_speed);
            while (true)
            {
                causal::VariableContext::set_current_time(effected_time_action_pair.first +
                                                          sim_horizon +
                                                          causal::VariableContext::get_time_step_size());
                FP_DATA_TYPE speed;
                if (lon_lin_vel->get_value(speed))
                {
                    sim_horizon += causal::VariableContext::get_time_step_size();
                    effected_outcome_action_pair.first.final_speed = speed;
                }
                else
                {
                    break;
                }
            }
        }
        else
        {
            sim_horizon = (*effected_time_action_pairs)[i + 1].first -
                    effected_time_action_pair.first;
            causal::VariableContext::set_current_time(effected_time_action_pair.first + sim_horizon);
            effected_fwd_car->get_lon_lin_vel_variable()->get_value(
                        effected_outcome_action_pair.first.final_speed);
        }
        causal::LaneEncapsulatingVariable lane_encaps(effected_fwd_car->get_pos_variable(), &map);
        causal::IdsPreviousTimeStepVariable prev_lane_encaps(&lane_encaps);
        causal::LaneTransitionsCalcVariable lane_trans(&prev_lane_encaps, &lane_encaps, &map);
        effected_outcome_action_pair.first.lane_transitions = 0.0;
        temporal::Time current_time;
        for (current_time = effected_time_action_pair.first +
             causal::VariableContext::get_time_step_size();
             current_time <= effected_time_action_pair.first + sim_horizon;
             current_time += causal::VariableContext::get_time_step_size())
        {
            causal::VariableContext::set_current_time(current_time);
            FP_DATA_TYPE lane_transitions;
            lane_trans.get_value(lane_transitions);
            effected_outcome_action_pair.first.lane_transitions += lane_transitions;
        }
        for (current_time = effected_time_action_pair.first;
             current_time <= effected_time_action_pair.first + sim_horizon;
             current_time += causal::VariableContext::get_time_step_size())
        {
            causal::VariableContext::set_current_time(current_time);
            geometry::Vec env_force;
            effected_fwd_car->get_env_force_variable()->get_value(env_force);
            if (current_time == effected_time_action_pair.first)
            {
                effected_outcome_action_pair.first.max_env_force_mag = env_force.norm();
            }
            else
            {
                effected_outcome_action_pair.first.max_env_force_mag = std::max(
                            env_force.norm(),
                            effected_outcome_action_pair.first.max_env_force_mag);
            }
        }
        effected_outcome_action_pair.first.action_done = true;
        effected_outcome_action_pair.second = effected_time_action_pair.second;

        // TODO: Integrate better information regarding braking
        agents::FullControlFWDCar effected_control_fwd_car(&map, 163 * 20, -163 * 20, 0.616);
        agents::DefaultFWDCarOutcomeSim original_outcome_sim(&effected_control_fwd_car,
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
        agents::GreedyPlanFWDCar effected_original_plan_fwd_car(&map, &original_outcome_sim,
                                                       outcome_sim_params, &reward_calc,
                                                       reward_calc_params, 0, 45, 2.5, 5, 2.5);
        effected_control_fwd_car.set_fwd_car(effected_fwd_car);
        effected_original_plan_fwd_car.set_control_fwd_car(&effected_control_fwd_car);

        causal::VariableContext::set_current_time(effected_time_action_pair.first);
        causal::IEndogenousVariable<agents::FWDCarOutcomeActionPair> *effected_original_fwd_car_best_action =
                effected_original_plan_fwd_car.get_best_outcome_action_pair_variable();
        effected_original_fwd_car_best_action->set_value(effected_outcome_action_pair);
        agents::FWDCarOutcomeActionPair effected_best_sim_outcome_action_pair;
        effected_original_fwd_car_best_action->get_value(effected_best_sim_outcome_action_pair);
        agents::FWDCarAction effected_best_sim_action = effected_best_sim_outcome_action_pair.second;
        causal::IEndogenousVariable<agents::FWDCarRewardParameters> *effected_original_fwd_car_reward_params =
                effected_original_plan_fwd_car.get_reward_params_variable();
        effected_original_fwd_car_reward_params->get_value(reward_calc_params);

        effected_control_fwd_car.set_fwd_car(nullptr);

        std::cout << "Reward Parameters: { lane_transition: " <<
                     reward_calc_params.lane_transitions_weight << ", caution: " <<
                     reward_calc_params.dist_headway_weight << ", speed_limit_excess: " <<
                     reward_calc_params.anti_speed_weight << ", max_env_force_mag: " <<
                     reward_calc_params.max_env_force_mag_weight << ", bias: " <<
                     reward_calc_params.bias_weight << " }" << std::endl;

        for (j = 0; j < ids->count(); ++j)
        {
            agents::FWDCar *causing_fwd_car = id_fwd_car_dict[(*ids)[j]];

            if ((*ids)[j] == effected_agent_id)
            {
                continue;
            }
            else
            {
                structures::IArray<agents::TimeFWDCarActionPair> *causing_time_action_pairs =
                        id_action_dict[(*ids)[j]];

                for (k = 1; k < causing_time_action_pairs->count(); ++k)
                {
                    agents::TimeFWDCarActionPair causing_time_action_pair = (*causing_time_action_pairs)[k];

                    if (causing_time_action_pair.first >= effected_time_action_pair.first)
                    {
                        continue;
                    }

                    //std::cout << "Testing link: Action " << k << " of agent " << (*ids)[j] <<
                    //             " -> Action " << i << " of agent " << effected_agent_id <<
                    //             std::endl;

                    //agents::RectRigidBodyEnv env;

                    /*
                    for (l = 0; l < ids->count(); ++l)
                    {
                        uint64_t id = (*ids)[l];
                        if (id != effected_agent_id && l != j)
                        {
                            env.add_rigid_body(id_fwd_car_dict[id]);
                        }
                    }
                    */


                    agents::FWDCarSim causing_fwd_car_sim(causing_fwd_car, causing_time_action_pair.first);

                    // TODO: Integrate better information regarding braking
                    agents::FullControlFWDCar causing_control_fwd_car(&map, 163 * 20, -163, 0.616);
                    agents::FullControlFWDCarSim causing_control_fwd_car_sim(
                                &causing_control_fwd_car, causing_time_action_pair.first);

                    agents::FWDCarActionIntervention causing_plan_fwd_car(default_fwd_car_action);

                    causal::IEndogenousVariable<agents::FWDCarAction> *causing_fwd_car_action_intervention =
                            causing_plan_fwd_car.get_action_intervention_variable();
                    for (l = 0; l < k; ++l)
                    {
                        agents::TimeFWDCarActionPair current_action = (*causing_time_action_pairs)[l];
                        for (temporal::Time current_time = current_action.first;
                             current_time <= end_time;
                             current_time += causal::VariableContext::get_time_step_size())
                        {
                            causal::VariableContext::set_current_time(current_time);
                            causing_fwd_car_action_intervention->set_value(current_action.second);
                        }
                    }

                    causing_control_fwd_car_sim.set_fwd_car(&causing_fwd_car_sim);
                    causing_plan_fwd_car.set_control_fwd_car(&causing_control_fwd_car_sim);

                    //env.add_rigid_body(&causing_fwd_car_sim);
                    original_env->remove_rigid_body(causing_fwd_car);
                    original_env->add_rigid_body(&causing_fwd_car_sim);


                    agents::FWDCarSim effected_fwd_car_sim(effected_fwd_car, effected_time_action_pair.first);

                    agents::FullControlFWDCarSim effected_control_fwd_car_sim(
                                &effected_control_fwd_car, effected_time_action_pair.first);

                    agents::DefaultFWDCarOutcomeSim outcome_sim(&effected_control_fwd_car_sim,
                                                                original_env);
                    agents::GreedyPlanFWDCar effected_plan_fwd_car(&map, &outcome_sim,
                                                                   outcome_sim_params, &reward_calc,
                                                                   reward_calc_params, 0, 45, 2.5,
                                                                   5, 2.5);

                    effected_control_fwd_car_sim.set_fwd_car(&effected_fwd_car_sim);
                    effected_plan_fwd_car.set_control_fwd_car(&effected_control_fwd_car_sim);

                    //env.add_rigid_body(&effected_fwd_car_sim);
                    original_env->remove_rigid_body(effected_fwd_car);
                    original_env->add_rigid_body(&effected_fwd_car_sim);


                    causal::VariableContext::set_current_time(effected_time_action_pair.first);
                    causal::IEndogenousVariable<agents::FWDCarOutcomeActionPair> *effected_fwd_car_best_action =
                                    effected_plan_fwd_car.get_best_outcome_action_pair_variable();
                    agents::FWDCarOutcomeActionPair effected_best_alt_sim_outcome_action_pair;
                    effected_fwd_car_best_action->get_value(effected_best_alt_sim_outcome_action_pair);
                    agents::FWDCarAction effected_best_alt_sim_action =
                            effected_best_alt_sim_outcome_action_pair.second;


                    original_env->remove_rigid_body(&effected_fwd_car_sim);
                    original_env->add_rigid_body(effected_fwd_car);

                    original_env->remove_rigid_body(&causing_fwd_car_sim);
                    original_env->add_rigid_body(causing_fwd_car);

                    // Compare effected_time_action_pair, effected_best_sim_action and effected_best_alt_sim_action

                    /*
                    std::cout << "Action Time: " <<
                                 std::to_string(effected_time_action_pair.first.time_since_epoch().count() /
                                                1000) << " s" << std::endl;
                    std::cout << "Effected Action: " << effected_time_action_pair.second << std::endl;
                    std::cout << "Effected Best Sim. Action: " << effected_best_sim_action << std::endl;
                    std::cout << "Effected Best Alt. Sim. Action: " << effected_best_alt_sim_action << std::endl;
                    */

                    FP_DATA_TYPE representativeness = diff(effected_time_action_pair.second,
                                                           effected_best_sim_action);
                    FP_DATA_TYPE alt_representativeness = diff(effected_time_action_pair.second,
                                                           effected_best_sim_action);
                    FP_DATA_TYPE intervention_impact = diff(effected_best_sim_action,
                                                            effected_best_alt_sim_action);

                    //std::cout << "Intervention Impact: " << intervention_impact << std::endl;

                    // TODO: Change this from being hardcoded to a command line argument
                    FP_DATA_TYPE intervention_impact_threshold = 0.25;

                    if (//representativeness < alt_representativeness &&
                            //representativeness < intervention_impact &&
                            intervention_impact > intervention_impact_threshold)
                    {
                        std::cout << "Action " << k << " of agent " << (*ids)[j] <<
                                     " -> Action " << i << " of agent " << effected_agent_id <<
                                     std::endl;
                        std::cout << "Action Time: " <<
                                     std::to_string(effected_time_action_pair.first.time_since_epoch().count() /
                                                    1000) << " s" << std::endl;
                        std::cout << "Effected Action: " << effected_time_action_pair.second << std::endl;
                        std::cout << "Effected Best Sim. Action: " << effected_best_sim_action << std::endl;
                        std::cout << "Effected Best Alt. Sim. Action: " << effected_best_alt_sim_action << std::endl;
                    }
                }
            }
        }
    }

    geometry::TrigBuff::destroy_instance();
}
