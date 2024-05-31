
#include <ori/simcars/structures/stl/stl_set.hpp>
#include <ori/simcars/geometry/trig_buff.hpp>
#include <ori/simcars/map/laneletd/laneletd_map.hpp>
#include <ori/simcars/causal/variable_context.hpp>
#include <ori/simcars/agents/fwd_car_action.hpp>
#include <ori/simcars/agents/fwd_car_action_extractor.hpp>
#include <ori/simcars/agents/fwd_car_sim.hpp>
#include <ori/simcars/agents/full_control_fwd_car_sim.hpp>
#include <ori/simcars/agents/fwd_car_action_intervention.hpp>
#include <ori/simcars/agents/otherd/otherd_fwd_car_scene.hpp>
#include <ori/simcars/visualisation/qmap_agents_widget.hpp>

#include <QApplication>
#include <QFrame>

#include <iostream>
#include <exception>

using namespace ori::simcars;

int main(int argc, char *argv[])
{
    if (argc < 9 || (argc > 9 && argc < 13))
    {
        std::cerr << "Usage: ./highd_intervention lanelet_map_file_path recording_meta_file_path "
                     "tracks_meta_file_path tracks_file_path start_frame end_frame agent_id "
                     "intervention_action [speed_goal_val speed_goal_time lane_goal_val "
                     "lane_goal_time]" <<
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

    agents::IFWDCarScene *scene;

    scene = new agents::otherd::OtherDFWDCarScene(argv[2], argv[3], argv[4], start_frame, end_frame);

    causal::VariableContext::set_time_step_size(scene->get_time_step_size());
    //causal::VariableContext::set_time_step_size(temporal::Duration(2000));

    structures::IArray<agents::FWDCar*> const *fwd_cars = scene->get_fwd_cars();

    std::cout << "Finished scene load" << std::endl;

    QFrame *frame = new QFrame();
    frame->setWindowTitle("SIMCARS Demo");
    frame->setFixedSize(1280, 1280);
    frame->show();

    visualisation::QMapAgentsWidget *map_scene_widget =
            new visualisation::QMapAgentsWidget(
                &map,
                frame,
                QPoint(10, 10),
                QSize(1260, 1260),
                scene->get_min_time(),
                scene->get_max_time(),
                std::chrono::milliseconds(40), 1.0, 3.0,
                visualisation::QMapAgentsWidget::FocusMode::FIXED);

    map_scene_widget->set_focal_position(map.get_map_centre());

    uint64_t agent_id = atoi(argv[7]);
    map_scene_widget->set_agent_colour(agent_id, sf::Color::Cyan);

    agents::FWDCarActionExtractor fwd_car_action_extractor(&map, temporal::Duration(500),
                                                           temporal::Duration(2500), 0.1,
                                                           1.0, temporal::Duration(500));

    agents::FWDCarSim *fwd_car_sim;

    // TODO: Integrate better information regarding braking
    agents::FullControlFWDCar control_fwd_car(&map, 163 * 20, -163 * 20, 0.616);
    agents::FullControlFWDCarSim *control_fwd_car_sim;

    agents::FWDCarAction default_fwd_car_action;

    if (argc > 9)
    {
        default_fwd_car_action.speed_goal.val = atof(argv[9]);
        default_fwd_car_action.speed_goal.time = temporal::Time(temporal::Duration(atoi(argv[10])));
        default_fwd_car_action.lane_goal.val = atoi(argv[11]);
        default_fwd_car_action.lane_goal.time = temporal::Time(temporal::Duration(atoi(argv[12])));
    }

    agents::FWDCarActionIntervention plan_fwd_car(default_fwd_car_action);
    causal::IEndogenousVariable<agents::FWDCarAction> *fwd_car_action_intervention =
            plan_fwd_car.get_action_intervention_variable();

    agents::RectRigidBodyEnv *env = scene->get_env();
    size_t i, j;
    for (i = 0; i < fwd_cars->count(); ++i)
    {
        agents::FWDCar *fwd_car = (*fwd_cars)[i];

        uint64_t id;
        bool res = fwd_car->get_id_variable()->get_value(id);

        if (res && agent_id == id)
        {
            structures::IArray<agents::TimeFWDCarActionPair> *fwd_car_actions =
                    fwd_car_action_extractor.extract_actions(fwd_car);

            size_t intervention_action = std::atoi(argv[8]);
            if (intervention_action >= fwd_car_actions->count())
            {
                throw std::runtime_error("Intervention action index too large");
            }

            temporal::Time intervention_time;
            for (j = 0; j < fwd_car_actions->count(); ++j)
            {
                if (j == intervention_action)
                {
                    intervention_time = (*fwd_car_actions)[j].first;
                }
                else if (j < intervention_action && argc == 9)
                {
                    temporal::Time end_time(temporal::Duration(
                                                end_frame *
                                                causal::VariableContext::get_time_step_size()));
                    for (temporal::Time current_time = (*fwd_car_actions)[j].first;
                         current_time <= end_time;
                         current_time += causal::VariableContext::get_time_step_size())
                    {
                        causal::VariableContext::set_current_time(current_time);
                        fwd_car_action_intervention->set_value((*fwd_car_actions)[j].second);
                    }
                }
            }

            fwd_car_sim = new agents::FWDCarSim(fwd_car, intervention_time);
            control_fwd_car_sim = new agents::FullControlFWDCarSim(&control_fwd_car,
                                                                   intervention_time);
            control_fwd_car_sim->set_fwd_car(fwd_car_sim);
            plan_fwd_car.set_control_fwd_car(control_fwd_car_sim);

            /*
             * TODO: Find a better way to handle the propogation of the pre-sim env. force once the
             * original rigid body has been removed from the env.
             */
            env->remove_rigid_body(fwd_car);
            env->add_rigid_body(fwd_car_sim);

            /*
             *  This section essentially just simulates the car ahead of time in order to prevent
             *  some inconsistencies that arise from making the visualisation itself call the
             *  simulation
             */
            causal::VariableContext::set_current_time(scene->get_max_time());

            agents::FWDCarOutcome outcome;

            control_fwd_car_sim->get_cumil_lane_trans_variable()->get_value(outcome.lane_transitions);
            fwd_car_sim->get_lon_lin_vel_variable()->get_value(outcome.final_speed);
            fwd_car_sim->get_max_env_force_mag_variable()->get_value(outcome.max_env_force_mag);

            causal::VariableContext::set_current_time(scene->get_min_time());

            map_scene_widget->insert(fwd_car_sim);

            delete fwd_car_actions;
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

    if (control_fwd_car_sim != nullptr) delete control_fwd_car_sim;
    if (fwd_car_sim != nullptr) delete fwd_car_sim;

    delete scene;

    geometry::TrigBuff::destroy_instance();

    return result;
}
