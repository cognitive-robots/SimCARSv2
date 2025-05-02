
#include <ori/simcars/structures/stl/stl_set.hpp>
#include <ori/simcars/geometry/trig_buff.hpp>
#include <ori/simcars/map/thor_magni/thor_magni_map.hpp>
#include <ori/simcars/causal/variable_context.hpp>
#include <ori/simcars/agents/ped_action.hpp>
#include <ori/simcars/agents/ped_action_extractor.hpp>
#include <ori/simcars/agents/ped_sim.hpp>
#include <ori/simcars/agents/goal_force_control_ped_sim.hpp>
#include <ori/simcars/agents/action_intervention_ped.hpp>
#include <ori/simcars/agents/default_ped_outcome_calc.hpp>
#include <ori/simcars/agents/default_ped_outcome_sim.hpp>
#include <ori/simcars/agents/default_ped_reward_calc.hpp>
#include <ori/simcars/agents/greedy_plan_ped.hpp>
#include <ori/simcars/agents/thor_magni/thor_magni_ped_scene.hpp>
#include <ori/simcars/visualisation/qped_map_agents_widget.hpp>

#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/ostreamwrapper.h>
#include <rapidjson/writer.h>

//#include <rapidcsv.h>

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
    if (argc < 13)
    {
        std::cerr << "Usage: ./thor_magni_sim_visualisation texture_file_path "
                     "offset_json_file_path scene_file_path goals_file_path start_frame "
                     "end_frame causing_agent_id causing_agent_action affected_agent_id "
                     "affected_agent_action task_op task_node_id" <<
                     std::endl;
        return -1;
    }

    size_t start_frame = atoi(argv[5]);
    size_t end_frame = atoi(argv[6]);

    geometry::TrigBuff::init_instance(360000, geometry::AngleType::RADIANS);

    QApplication app(argc, argv);

    std::cout << "Beginning map load" << std::endl;

    map::thor_magni::ThorMagniMap map;

    map.load(argv[1], argv[2], argv[3], argv[4]);

    std::cout << "Finished map load" << std::endl;


    std::cout << "Beginning scene load" << std::endl;

    agents::IPedScene *scene =
            new agents::thor_magni::ThorMagniPedScene(argv[3], start_frame, end_frame);

    causal::VariableContext::set_time_step_size(scene->get_time_step_size());

    structures::IArray<agents::Ped*> const *peds = scene->get_peds();

    std::cout << "Finished scene load" << std::endl;


    agents::PedActionExtractor ped_action_extractor(&map, temporal::Duration(200),
                                                    temporal::Duration(200),
                                                    temporal::Duration(200));

    structures::stl::STLDictionary<uint64_t, agents::Ped*> id_ped_dict;
    structures::stl::STLDictionary<uint64_t, structures::IArray<agents::TimePedActionPair>*> id_action_dict;

    size_t u;
    for (u = 0; u < peds->count(); ++u)
    {
        agents::Ped *ped = (*peds)[u];

        uint64_t id;
        bool res = ped->get_id_variable()->get_value(id);

        if (!res)
        {
            throw std::runtime_error("Could not get ped id");
        }

        //std::cout << "Extracting actions for agent " << id << std::endl;

        try
        {
            structures::IArray<agents::TimePedActionPair> *ped_actions =
                    ped_action_extractor.extract_actions(ped);

            id_ped_dict.update(id, ped);
            id_action_dict.update(id, ped_actions);
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

    agents::PedTask::TaskOp task_op = agents::PedTask::TaskOp(std::atoi(argv[11]));
    uint64_t task_node_id = std::atoll(argv[12]);
    agents::PedTask task(task_op, task_node_id);

    agents::Ped *affected_ped = id_ped_dict[affected_agent_id];
    structures::IArray<agents::TimePedActionPair> *affected_time_action_pairs =
            id_action_dict[affected_agent_id];

    agents::PointMassEnv *original_env = scene->get_env();

    agents::TimePedActionPair affected_time_action_pair =
            (*affected_time_action_pairs)[affected_agent_action];

    agents::PedOutcomeActionPair affected_outcome_action_pair;
    temporal::Duration sim_horizon(0);
    if (affected_agent_action == affected_time_action_pairs->count() - 1)
    {
        causal::IEndogenousVariable<geometry::Vec> *pos = affected_ped->get_pos_variable();
        causal::VariableContext::set_current_time(affected_time_action_pair.first);
        pos->get_value(affected_outcome_action_pair.first.pos);
        while (true)
        {
            causal::VariableContext::set_current_time(
                        affected_time_action_pair.first + sim_horizon +
                        causal::VariableContext::get_time_step_size());
            geometry::Vec position;
            if (pos->get_value(position))
            {
                sim_horizon += causal::VariableContext::get_time_step_size();
                affected_outcome_action_pair.first.pos = position;
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
        affected_ped->get_pos_variable()->get_value(affected_outcome_action_pair.first.pos);
    }
    affected_ped->get_min_neighbour_dist_variable()->get_value(
                affected_outcome_action_pair.first.min_neighbour_dist);
    affected_outcome_action_pair.first.action_done = true;
    affected_outcome_action_pair.second = affected_time_action_pair.second;

    // TODO: Integrate better information regarding braking
    agents::GoalForceControlPed affected_control_ped(&map, 200.0);
    agents::DefaultPedOutcomeCalc original_outcome_calc(&affected_control_ped);
    agents::DefaultPedOutcomeSim original_outcome_sim(&affected_control_ped,
                                                      original_env);
    agents::PedSimParameters outcome_sim_params = {
        .sim_horizon_secs = std::chrono::duration_cast<std::chrono::duration<FP_DATA_TYPE>>(
        sim_horizon).count()
    };
    // Cannot be set during initialisation due to this parameter belonging to a base class
    outcome_sim_params.action_done_node_dist_threshold = 0.375;
    agents::DefaultPedRewardCalc reward_calc(&map);
    agents::PedRewardParameters reward_calc_params = {
        .task_goal_weight = 0.5,
        .space_weight = 0.5,
        .bias_weight = 0.0
    };
    agents::GreedyPlanPed affected_original_plan_ped(&map, &original_outcome_sim,
                                                     &original_outcome_calc, outcome_sim_params,
                                                     &reward_calc, reward_calc_params, 5, 2.5);
    affected_control_ped.set_ped(affected_ped);
    affected_original_plan_ped.set_control_ped(&affected_control_ped);

    affected_original_plan_ped.set_task(task);

    causal::VariableContext::set_current_time(affected_time_action_pair.first);
    causal::IEndogenousVariable<agents::PedOutcomeActionPair> *affected_original_ped_best_action =
            affected_original_plan_ped.get_best_outcome_action_pair_variable();
    //affected_original_ped_best_action->set_value(affected_outcome_action_pair);
    agents::PedOutcomeActionPair affected_best_sim_outcome_action_pair;
    affected_original_ped_best_action->get_value(affected_best_sim_outcome_action_pair);
    agents::PedAction affected_best_sim_action = affected_best_sim_outcome_action_pair.second;
    //causal::IEndogenousVariable<agents::PedRewardParameters> *affected_original_ped_reward_params =
    //        affected_original_plan_ped.get_reward_params_variable();
    //affected_original_ped_reward_params->get_value(reward_calc_params);

    affected_control_ped.set_ped(nullptr);

    agents::PedSim affected_ped_sim(affected_ped, affected_time_action_pair.first);
    agents::GoalForceControlPedSim affected_control_ped_sim(
                &affected_control_ped, affected_time_action_pair.first -
                causal::VariableContext::get_time_step_size());
    agents::ActionInterventionPed affected_best_alt_sim_action_intervention(
                affected_best_sim_action);
    affected_control_ped_sim.set_ped(&affected_ped_sim);
    affected_best_alt_sim_action_intervention.set_control_ped(&affected_control_ped_sim);

    original_env->remove_point_mass(affected_ped);
    original_env->add_point_mass(&affected_ped_sim);

    /*
     *  This section essentially just simulates the car ahead of time in order to prevent
     *  some inconsistencies that arise from making the visualisation itself call the
     *  simulation
     */
    /*
    if (argc > 13)
    {
        std::string const output_csv_file_path_str(argv[13]);
        rapidcsv::Document output_csv_document;

        std::vector<FP_DATA_TYPE> time_vector;
        std::vector<FP_DATA_TYPE> magenta_reward_vector;

        causal::IEndogenousVariable<FP_DATA_TYPE> *magenta_reward_variable =
                affected_original_plan_ped.get_real_reward_variable();

        FP_DATA_TYPE magenta_reward;
        temporal::Time time;
        for (time = scene->get_min_time(); time <= scene->get_max_time();
             time += scene->get_time_step_size())
        {
            causal::VariableContext::set_current_time(time);

            magenta_reward_variable->get_value(magenta_reward);

            time_vector.push_back(std::chrono::duration_cast<std::chrono::duration<FP_DATA_TYPE>>(
                                      time.time_since_epoch()).count());
            magenta_reward_vector.push_back(magenta_reward);
        }

        output_csv_document.InsertColumn(0, time_vector, "time");
        output_csv_document.InsertColumn(1, magenta_reward_vector, "magenta_reward");

        output_csv_document.Save(output_csv_file_path_str);
    }
    else
    {
    */
    causal::VariableContext::set_current_time(scene->get_max_time());

    agents::PedOutcome outcome;

    affected_ped_sim.get_pos_variable()->get_value(outcome.pos);
    affected_ped_sim.get_min_neighbour_dist_variable()->get_value(outcome.min_neighbour_dist);
    //}

    causal::VariableContext::set_current_time(scene->get_min_time());

    QFrame *frame = new QFrame();
    frame->setWindowTitle("SIMCARS Demo");
    frame->setFixedSize(1280, 1280);
    frame->show();

    visualisation::QPedMapAgentsWidget *map_scene_widget =
            new visualisation::QPedMapAgentsWidget(
                &map,
                frame,
                QPoint(10, 10),
                QSize(1260, 1260),
                scene->get_min_time(),
                scene->get_max_time(),
                std::chrono::milliseconds(40), 1.0, 1260.0 / map.get_max_dim_size(),
                visualisation::QPedMapAgentsWidget::FocusMode::FIXED);

    map_scene_widget->set_focal_position(map.get_map_centre());

    map_scene_widget->set_agent_colour(causing_agent_id, sf::Color::Cyan);
    map_scene_widget->set_agent_colour(affected_agent_id, sf::Color::Magenta);

    for (size_t i = 13; i < argc; ++i)
    {
        uint64_t other_rel_agent_id = std::atoll(argv[i]);
        map_scene_widget->set_agent_colour(other_rel_agent_id, sf::Color::Yellow);
    }

    for (size_t i = 0; i < peds->count(); ++i)
    {
        agents::Ped *ped = (*peds)[i];

        uint64_t id;
        bool res = ped->get_id_variable()->get_value(id);

        if (!res)
        {
            throw std::runtime_error("Could not get ped id");
        }

        if (id == affected_agent_id)
        {
            map_scene_widget->insert(&affected_ped_sim);
        }
        else
        {
            map_scene_widget->insert(ped);
        }
    }

    map_scene_widget->show();

    int result = app.exec();

    delete map_scene_widget;

    delete frame;

    geometry::TrigBuff::destroy_instance();

    return result;
}
