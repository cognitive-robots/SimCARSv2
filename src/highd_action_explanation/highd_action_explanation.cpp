
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

#include <QApplication>
#include <QFrame>

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

    //causal::VariableContext::set_time_step_size(scene.get_time_step_size());
    causal::VariableContext::set_time_step_size(temporal::Duration(100));

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

        std::cout << "Extracting actions for agent " << id << std::endl;

        id_fwd_car_dict.update(id, fwd_car);

        structures::IArray<agents::TimeFWDCarActionPair> *fwd_car_actions =
                fwd_car_action_extractor.extract_actions(fwd_car);

        id_action_dict.update(id, fwd_car_actions);
    }

    uint64_t effected_agent_id = atoi(argv[6]);
    agents::FWDCar *effected_fwd_car = id_fwd_car_dict[effected_agent_id];
    structures::IArray<agents::TimeFWDCarActionPair> *effected_actions = id_action_dict[effected_agent_id];

    structures::IArray<uint64_t> const *ids = id_action_dict.get_keys();

    agents::FWDCarAction default_fwd_car_action;
    agents::RectRigidBodyEnv *original_env = scene.get_env();
    temporal::Time end_time = scene.get_max_time();
    size_t j, k, l;
    for (i = 1; i < effected_actions->count(); ++i)
    {
        std::cout << "Processing explanation for action " << i << " of agent " <<
                     effected_agent_id << std::endl;

        agents::TimeFWDCarActionPair effected_action = (*effected_actions)[i];

        // TODO: Integrate better information regarding braking
        agents::FullControlFWDCar effected_control_fwd_car(&map, 163 * 20, -163, 0.616);

        agents::DefaultFWDCarOutcomeSim original_outcome_sim(&effected_control_fwd_car,
                                                             original_env);
        agents::FWDCarSimParameters outcome_sim_params = { .sim_horizon_secs = 5 };
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

        causal::VariableContext::set_current_time(effected_action.first);
        causal::IEndogenousVariable<agents::FWDCarAction> *effected_original_fwd_car_best_action =
                effected_original_plan_fwd_car.get_best_action_variable();
        effected_original_fwd_car_best_action->set_value(effected_action.second);
        agents::FWDCarAction effected_closest_action;
        effected_original_fwd_car_best_action->get_value(effected_closest_action);
        causal::IEndogenousVariable<agents::FWDCarRewardParameters> *effected_original_fwd_car_reward_params =
                effected_original_plan_fwd_car.get_reward_params_variable();
        effected_original_fwd_car_reward_params->get_value(reward_calc_params);

        std::cout << "Reward Parameters: { lane_transition: " <<
                     reward_calc_params.lane_transitions_weight << ", caution: " <<
                     reward_calc_params.caution_weight << ", speed_limit_excess: " <<
                     reward_calc_params.speed_limit_excess_weight << ", max_env_force_mag: " <<
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
                structures::IArray<agents::TimeFWDCarActionPair> *causing_actions =
                        id_action_dict[(*ids)[j]];

                for (k = 1; k < causing_actions->count(); ++k)
                {
                    agents::TimeFWDCarActionPair causing_action = (*causing_actions)[k];

                    if (causing_action.first >= effected_action.first)
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


                    agents::FWDCarSim causing_fwd_car_sim(causing_fwd_car, causing_action.first);

                    // TODO: Integrate better information regarding braking
                    agents::FullControlFWDCar causing_control_fwd_car(&map, 163 * 20, -163, 0.616);
                    agents::FullControlFWDCarSim causing_control_fwd_car_sim(
                                &causing_control_fwd_car, causing_action.first);

                    agents::FWDCarActionIntervention causing_plan_fwd_car(default_fwd_car_action);

                    causal::IEndogenousVariable<agents::FWDCarAction> *causing_fwd_car_action_intervention =
                            causing_plan_fwd_car.get_action_intervention_variable();
                    for (l = 0; l < k; ++l)
                    {
                        agents::TimeFWDCarActionPair current_action = (*causing_actions)[l];
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


                    agents::FWDCarSim effected_fwd_car_sim(effected_fwd_car, effected_action.first);

                    agents::FullControlFWDCarSim effected_control_fwd_car_sim(
                                &effected_control_fwd_car, effected_action.first);

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


                    causal::VariableContext::set_current_time(effected_action.first);
                    causal::IEndogenousVariable<agents::FWDCarAction> * effected_fwd_car_best_action =
                                    effected_plan_fwd_car.get_best_action_variable();
                    agents::FWDCarAction effected_alt_action;
                    effected_fwd_car_best_action->get_value(effected_alt_action);


                    original_env->remove_rigid_body(&effected_fwd_car_sim);
                    original_env->add_rigid_body(effected_fwd_car);

                    original_env->remove_rigid_body(&causing_fwd_car_sim);
                    original_env->add_rigid_body(causing_fwd_car);

                    // Compare effected_action, effected_closest_action and effected_alt_action

                    std::cout << "Action Time: " <<
                                 std::to_string(effected_action.first.time_since_epoch().count() /
                                                1000) << " s" << std::endl;
                    std::cout << "Effected Action: " << effected_action.second << std::endl;
                    std::cout << "Effected Closest Action: " << effected_closest_action << std::endl;
                    std::cout << "Effected Closest Alt. Action: " << effected_alt_action << std::endl;

                    FP_DATA_TYPE representativeness = diff(effected_action.second,
                                                           effected_closest_action);
                    FP_DATA_TYPE alt_representativeness = diff(effected_action.second,
                                                           effected_closest_action);
                    FP_DATA_TYPE intervention_impact = diff(effected_closest_action,
                                                            effected_alt_action);

                    std::cout << "Intervention Impact: " << intervention_impact << std::endl;

                    // TODO: Change this from being hardcoded to a command line argument
                    FP_DATA_TYPE intervention_impact_threshold = 0.25;

                    if (//representativeness < alt_representativeness &&
                            //representativeness < intervention_impact &&
                            intervention_impact > intervention_impact_threshold)
                    {
                        std::cout << "Action " << k << " of agent " << (*ids)[j] <<
                                     " -> Action " << i << " of agent " << effected_agent_id <<
                                     std::endl;
                    }
                }
            }
        }
    }

    geometry::TrigBuff::destroy_instance();
}
