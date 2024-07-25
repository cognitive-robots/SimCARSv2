
#include <ori/simcars/structures/stl/stl_set.hpp>
#include <ori/simcars/geometry/trig_buff.hpp>
#include <ori/simcars/map/laneletd/laneletd_map.hpp>
#include <ori/simcars/causal/variable_context.hpp>
#include <ori/simcars/agents/fwd_car_action.hpp>
#include <ori/simcars/agents/fwd_car_action_extractor.hpp>
#include <ori/simcars/agents/fwd_car_sim.hpp>
#include <ori/simcars/agents/full_control_fwd_car_sim.hpp>
#include <ori/simcars/agents/action_intervention_fwd_car.hpp>
#include <ori/simcars/agents/default_fwd_car_outcome_sim.hpp>
#include <ori/simcars/agents/default_fwd_car_reward_calc.hpp>
#include <ori/simcars/agents/greedy_plan_fwd_car.hpp>
#include <ori/simcars/agents/otherd/otherd_fwd_car_scene.hpp>
#include <ori/simcars/visualisation/qdriving_map_agents_widget.hpp>

#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/ostreamwrapper.h>
#include <rapidjson/writer.h>

#include <QApplication>
#include <QFrame>

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
    if (argc < 11)
    {
        std::cerr << "Usage: ./laneletd_sim_visualisation lanelet_map_file_path "
                     "recording_meta_file_path tracks_meta_file_path tracks_file_path start_frame "
                     "end_frame causing_agent_id causing_agent_action affected_agent_id "
                     "affected_agent_action" <<
                     std::endl;
        return -1;
    }

    size_t start_frame = atoi(argv[5]);
    size_t end_frame = atoi(argv[6]);

    geometry::TrigBuff::init_instance(360000, geometry::AngleType::RADIANS);

    QApplication app(argc, argv);

    std::cout << "Beginning map load" << std::endl;

    map::laneletd::LaneletDMap map;

    map.load(argv[1], argv[2]);

    std::cout << "Finished map load" << std::endl;


    std::cout << "Beginning scene load" << std::endl;

    agents::otherd::OtherDFWDCarScene scene(argv[2], argv[3], argv[4], start_frame, end_frame);

    causal::VariableContext::set_time_step_size(scene.get_time_step_size());

    structures::IArray<agents::FWDCar*> const *fwd_cars = scene.get_fwd_cars();

    std::cout << "Finished scene load" << std::endl;


    agents::FWDCarActionExtractor fwd_car_action_extractor(&map, temporal::Duration(500),
                                                           temporal::Duration(2500), 0.1,
                                                           1.0, temporal::Duration(500));

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

        try
        {
            structures::IArray<agents::TimeFWDCarActionPair> *fwd_car_actions =
                    fwd_car_action_extractor.extract_actions(fwd_car);

            id_fwd_car_dict.update(id, fwd_car);
            id_action_dict.update(id, fwd_car_actions);
        }
        catch (std::runtime_error)
        {
            // We don't need to do anything, we just won't be able to consider the actions for this
            // agent
        }
    }

    uint64_t causing_agent_id = std::atoll(argv[7]);
    uint64_t affected_agent_id = std::atoll(argv[9]);
    uint64_t affected_agent_action = std::atoll(argv[10]);

    agents::FWDCar *affected_fwd_car = id_fwd_car_dict[affected_agent_id];
    structures::IArray<agents::TimeFWDCarActionPair> *affected_time_action_pairs =
            id_action_dict[affected_agent_id];

    agents::RectRigidBodyEnv *original_env = scene.get_env();

    agents::TimeFWDCarActionPair affected_time_action_pair = (*affected_time_action_pairs)[affected_agent_action];

    agents::FWDCarOutcomeActionPair affected_outcome_action_pair;
    temporal::Duration sim_horizon(0);
    if (affected_agent_action == affected_time_action_pairs->count() - 1)
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
        sim_horizon = (*affected_time_action_pairs)[affected_agent_action + 1].first -
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

    agents::FWDCarSim affected_fwd_car_sim(affected_fwd_car, affected_time_action_pair.first);
    agents::FullControlFWDCarSim affected_control_fwd_car_sim(
                &affected_control_fwd_car, affected_time_action_pair.first);
    agents::ActionInterventionFWDCar affected_best_alt_sim_action_intervention(
                affected_best_sim_action);
    affected_control_fwd_car_sim.set_fwd_car(&affected_fwd_car_sim);
    affected_best_alt_sim_action_intervention.set_control_fwd_car(&affected_control_fwd_car_sim);

    original_env->remove_rigid_body(affected_fwd_car);
    original_env->add_rigid_body(&affected_fwd_car_sim);

    /*
     *  This section essentially just simulates the car ahead of time in order to prevent
     *  some inconsistencies that arise from making the visualisation itself call the
     *  simulation
     */
    causal::VariableContext::set_current_time(scene.get_max_time());

    agents::FWDCarOutcome outcome;

    affected_control_fwd_car_sim.get_cumil_lane_trans_variable()->get_value(outcome.lane_transitions);
    affected_fwd_car_sim.get_lon_lin_vel_variable()->get_value(outcome.final_speed);
    affected_fwd_car_sim.get_dist_headway_variable()->get_value(outcome.dist_headway);
    affected_fwd_car_sim.get_max_env_force_mag_variable()->get_value(outcome.max_env_force_mag);

    causal::VariableContext::set_current_time(scene.get_min_time());

    QFrame *frame = new QFrame();
    frame->setWindowTitle("SIMCARS Demo");
    frame->setFixedSize(1280, 1280);
    frame->show();

    visualisation::QDrivingMapAgentsWidget *map_scene_widget =
            new visualisation::QDrivingMapAgentsWidget(
                &map,
                frame,
                QPoint(10, 10),
                QSize(1260, 1260),
                scene.get_min_time(),
                scene.get_max_time(),
                std::chrono::milliseconds(40), 1.0, 1260.0 / map.get_max_dim_size(),
                visualisation::QDrivingMapAgentsWidget::FocusMode::FIXED);

    map_scene_widget->set_focal_position(map.get_map_centre());

    map_scene_widget->set_agent_colour(causing_agent_id, sf::Color::Cyan);
    map_scene_widget->set_agent_colour(affected_agent_id, sf::Color::Magenta);

    for (size_t i = 11; i < argc; ++i)
    {
        uint64_t other_rel_agent_id = std::atoll(argv[i]);
        map_scene_widget->set_agent_colour(other_rel_agent_id, sf::Color::Yellow);
    }

    for (size_t i = 0; i < fwd_cars->count(); ++i)
    {
        agents::FWDCar *fwd_car = (*fwd_cars)[i];

        uint64_t id;
        bool res = fwd_car->get_id_variable()->get_value(id);

        if (!res)
        {
            throw std::runtime_error("Could not get FWD car id");
        }

        if (id == affected_agent_id)
        {
            map_scene_widget->insert(&affected_fwd_car_sim);
        }
        else
        {
            map_scene_widget->insert(fwd_car);
        }
    }

    map_scene_widget->show();

    int result = app.exec();

    delete map_scene_widget;

    delete frame;

    geometry::TrigBuff::destroy_instance();

    return result;
}
