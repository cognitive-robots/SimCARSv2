
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

#include <rapidcsv.h>

#include <QApplication>
#include <QFrame>

#include <iostream>
#include <exception>

using namespace ori::simcars;

int main(int argc, char *argv[])
{
    if (argc < 14)
    {
        std::cerr << "Usage: ./thor_magni_alt_visualisation_with_reward_output texture_file_path "
                     "offset_json_file_path scene_file_path goals_file_path start_frame "
                     "end_frame causing_agent_id causing_agent_action affected_agent_id "
                     "affected_agent_action task_op task_node_id output_csv_file_path" <<
                     std::endl;
        return -1;
    }

    QApplication app(argc, argv);

    std::cout << "Beginning map load" << std::endl;

    map::thor_magni::ThorMagniMap map;

    map.load(argv[1], argv[2], argv[3], argv[4]);

    std::cout << "Finished map load" << std::endl;

    std::cout << "Beginning scene load" << std::endl;

    agents::IPedScene *scene;

    if (argc > 5)
    {
        size_t start_frame = atoi(argv[5]);
        size_t end_frame = atoi(argv[6]);
        scene = new agents::thor_magni::ThorMagniPedScene(argv[3], start_frame, end_frame);
    }
    else
    {
        scene = new agents::thor_magni::ThorMagniPedScene(argv[3]);
    }

    structures::IArray<agents::Ped*> const *agents = scene->get_peds();

    std::cout << "Finished scene load" << std::endl;


    agents::PedActionExtractor ped_action_extractor(&map, temporal::Duration(200),
                                                    temporal::Duration(200),
                                                    temporal::Duration(200));

    structures::stl::STLDictionary<uint64_t, agents::Ped*> id_ped_dict;
    structures::stl::STLDictionary<uint64_t, structures::IArray<agents::TimePedActionPair>*> id_action_dict;

    structures::IArray<agents::Ped*> const *peds = scene->get_peds();

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
    uint64_t causing_agent_action = std::atoll(argv[10]);
    uint64_t affected_agent_id = std::atoll(argv[9]);
    uint64_t affected_agent_action = std::atoll(argv[10]);

    structures::IArray<agents::TimePedActionPair> *affected_agent_actions =
            id_action_dict[affected_agent_id];
    if (affected_agent_action >= affected_agent_actions->count())
    {
        std::cerr << "Affected agent action index too high" << std::endl;
        return -1;
    }
    if (affected_agent_action < 1)
    {
        std::cerr << "Affected agent action index too low" << std::endl;
        return -1;
    }

    agents::PedTask::TaskOp task_op = agents::PedTask::TaskOp(std::atoi(argv[11]));
    uint64_t task_node_id = std::atoll(argv[12]);
    agents::PedTask task(task_op, task_node_id);

    agents::Ped *causing_ped = id_ped_dict[causing_agent_id];
    agents::Ped *affected_ped = id_ped_dict[affected_agent_id];

    agents::PointMassEnv *env = scene->get_env();


    temporal::Time sim_start_time =
            (*id_action_dict[affected_agent_id])[affected_agent_action - 1].second.node_goal.time;
    temporal::Time sim_end_time =
            (*id_action_dict[affected_agent_id])[affected_agent_action].first;
    agents::PedSimParameters outcome_sim_params = {
        .sim_horizon_secs = std::chrono::duration_cast<std::chrono::duration<FP_DATA_TYPE>>(
        sim_end_time - sim_start_time).count()
    };
    // Cannot be set during initialisation due to this parameter belonging to a base class
    outcome_sim_params.action_done_node_dist_threshold = 0.375;
    agents::DefaultPedRewardCalc reward_calc(&map);
    agents::PedRewardParameters reward_calc_params = {
        .task_goal_weight = 0.8,
        .space_weight = 0.2,
        .bias_weight = 0.0
    };


    agents::GoalForceControlPed causing_control_ped(&map, 200.0);
    agents::DefaultPedOutcomeCalc causing_outcome_calc(&causing_control_ped);
    agents::DefaultPedOutcomeSim causing_outcome_sim(&causing_control_ped, env);
    agents::GreedyPlanPed causing_plan_ped(&map, &causing_outcome_sim,
                                                    &causing_outcome_calc,
                                                    outcome_sim_params, &reward_calc,
                                                    reward_calc_params, 5, 2.5);
    causing_control_ped.set_ped(causing_ped);
    causing_plan_ped.set_control_ped(&causing_control_ped);

    causing_plan_ped.set_task(task);


    agents::PedSim affected_ped_sim(affected_ped, sim_start_time);
    agents::GoalForceControlPed affected_control_ped(&map, 200.0);
    agents::GoalForceControlPedSim affected_control_ped_sim (
                &affected_control_ped, sim_start_time -
                causal::VariableContext::get_time_step_size());
    agents::DefaultPedOutcomeCalc affected_outcome_calc(&affected_control_ped_sim);
    agents::DefaultPedOutcomeSim affected_outcome_sim(&affected_control_ped_sim,
                                                               env);
    agents::GreedyPlanPed affected_alt_plan_ped(&map, &affected_outcome_sim,
                                                     &affected_outcome_calc,
                                                     outcome_sim_params, &reward_calc,
                                                     reward_calc_params, 5, 2.5);
    affected_control_ped_sim.set_ped(&affected_ped_sim);
    affected_alt_plan_ped.set_control_ped(&affected_control_ped_sim);

    affected_alt_plan_ped.set_task(task);

    env->remove_point_mass(affected_ped);
    env->add_point_mass(&affected_ped_sim);

    causal::VariableContext::set_current_time(sim_start_time);
    causal::IEndogenousVariable<agents::PedOutcomeActionPair> *affected_ped_best_action =
            affected_alt_plan_ped.get_best_outcome_action_pair_variable();
    agents::PedOutcomeActionPair affected_best_alt_sim_outcome_action_pair;
    affected_ped_best_action->get_value(affected_best_alt_sim_outcome_action_pair);
    agents::PedAction affected_best_alt_sim_action =
            affected_best_alt_sim_outcome_action_pair.second;

    agents::ActionInterventionPed affected_best_alt_sim_action_intervention(
                affected_best_alt_sim_action);
    affected_best_alt_sim_action_intervention.set_control_ped(&affected_control_ped_sim);


    std::string const output_csv_file_path_str(argv[13]);
    rapidcsv::Document output_csv_document;

    std::vector<FP_DATA_TYPE> time_vector;
    std::vector<FP_DATA_TYPE> cyan_overall_reward_vector;
    std::vector<FP_DATA_TYPE> cyan_task_goal_reward_vector;
    std::vector<FP_DATA_TYPE> cyan_space_reward_vector;
    std::vector<FP_DATA_TYPE> magenta_overall_reward_vector;
    std::vector<FP_DATA_TYPE> magenta_task_goal_reward_vector;
    std::vector<FP_DATA_TYPE> magenta_space_reward_vector;

    causal::IEndogenousVariable<FP_DATA_TYPE> *cyan_reward_variable =
            causing_plan_ped.get_real_reward_variable();
    causal::IEndogenousVariable<agents::PedRewards> *cyan_reward_metrics_variable =
            causing_plan_ped.get_real_reward_metrics_variable();
    causal::IEndogenousVariable<FP_DATA_TYPE> *magenta_reward_variable =
            affected_alt_plan_ped.get_real_reward_variable();
    causal::IEndogenousVariable<agents::PedRewards> *magenta_reward_metrics_variable =
            affected_alt_plan_ped.get_real_reward_metrics_variable();

    FP_DATA_TYPE cyan_reward;
    agents::PedRewards cyan_reward_metrics;
    FP_DATA_TYPE magenta_reward;
    agents::PedRewards magenta_reward_metrics;
    temporal::Time time;
    for (time = scene->get_min_time(); time <= scene->get_max_time();
         time += scene->get_time_step_size())
    {
        causal::VariableContext::set_current_time(time);

        cyan_reward_variable->get_value(cyan_reward);
        cyan_reward_metrics_variable->get_value(cyan_reward_metrics);
        magenta_reward_variable->get_value(magenta_reward);
        magenta_reward_metrics_variable->get_value(magenta_reward_metrics);

        time_vector.push_back(std::chrono::duration_cast<std::chrono::duration<FP_DATA_TYPE>>(
                                  time.time_since_epoch()).count());
        cyan_overall_reward_vector.push_back(cyan_reward);
        cyan_task_goal_reward_vector.push_back(cyan_reward_metrics.task_goal_reward);
        cyan_space_reward_vector.push_back(cyan_reward_metrics.space_reward);
        magenta_overall_reward_vector.push_back(magenta_reward);
        magenta_task_goal_reward_vector.push_back(magenta_reward_metrics.task_goal_reward);
        magenta_space_reward_vector.push_back(magenta_reward_metrics.space_reward);
    }

    output_csv_document.InsertColumn(0, time_vector, "time");
    output_csv_document.InsertColumn(1, cyan_overall_reward_vector, "cyan_overall_reward");
    output_csv_document.InsertColumn(2, cyan_task_goal_reward_vector, "cyan_task_goal_reward");
    output_csv_document.InsertColumn(3, cyan_space_reward_vector, "cyan_space_reward");
    output_csv_document.InsertColumn(4, magenta_overall_reward_vector, "magenta_overall_reward");
    output_csv_document.InsertColumn(5, magenta_task_goal_reward_vector, "magenta_task_goal_reward");
    output_csv_document.InsertColumn(6, magenta_space_reward_vector, "magenta_space_reward");

    output_csv_document.Save(output_csv_file_path_str);

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

    for (size_t i = 14; i < argc; ++i)
    {
        uint64_t other_rel_agent_id = std::atoll(argv[i]);
        map_scene_widget->set_agent_colour(other_rel_agent_id, sf::Color::Yellow);
    }

    for (size_t i = 0; i < agents->count(); ++i)
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

    delete scene;

    return result;
}
