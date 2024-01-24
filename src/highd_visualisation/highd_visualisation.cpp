
#include <ori/simcars/structures/stl/stl_set.hpp>
#include <ori/simcars/geometry/trig_buff.hpp>
#include <ori/simcars/map/highd/highd_map.hpp>
#include <ori/simcars/causal/variable_context.hpp>
#include <ori/simcars/agents/fwd_car_sim.hpp>
#include <ori/simcars/agents/highd/highd_fwd_car_scene.hpp>
#include <ori/simcars/visualisation/qmap_agents_widget.hpp>

#include <QApplication>
#include <QFrame>

#include <iostream>
#include <exception>

#define NUMBER_OF_AGENTS 30

using namespace ori::simcars;

int main(int argc, char *argv[])
{
    if (argc < 6)
    {
        std::cerr << "Usage: ./highd_simcars_demo recording_meta_file_path tracks_meta_file_path "
                     "tracks_file_path start_frame end_frame [agent_id] [intervention_frame]" <<
                     std::endl;
        return -1;
    }

    size_t start_frame = atoi(argv[4]);
    size_t end_frame = atoi(argv[5]);

    geometry::TrigBuff::init_instance(360000, geometry::AngleType::RADIANS);

    QApplication app(argc, argv);

    std::cout << "Beginning map load" << std::endl;

    map::highd::HighDMap map;

    map.load(argv[1]);

    std::cout << "Finished map load" << std::endl;

    std::cout << "Beginning scene load" << std::endl;

    agents::IFWDCarScene *scene;

    scene = new agents::highd::HighDFWDCarScene(argv[2], argv[3], start_frame, end_frame);

    structures::IArray<agents::FWDCar*> const *fwd_cars = scene->get_fwd_cars();

    std::cout << "Finished scene load" << std::endl;

    QFrame *frame = new QFrame();
    frame->setWindowTitle("SIMCARS Demo");
    frame->setFixedSize(1280, 1280);
    frame->show();

    geometry::Vec focal_position(210.0f, 19.0f);

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

    map_scene_widget->set_focal_position(focal_position);

    uint64_t agent_id;
    if (argc > 6)
    {
        agent_id = atoi(argv[6]);
        map_scene_widget->set_agent_colour(agent_id, sf::Color::Cyan);
    }

    agents::RectRigidBodyEnv *env = scene->get_env();
    structures::stl::STLDictionary<uint64_t, agents::FWDCarSim*> id_fwd_car_sim_dict;
    size_t i;
    for (i = 0; i < fwd_cars->count(); ++i)
    {
        agents::FWDCar *fwd_car = (*fwd_cars)[i];

        uint64_t id;
        bool res = fwd_car->get_id_variable()->get_value(id);

        if (argc > 7 && res && agent_id == id)
        {
            size_t intervention_frame = atoi(argv[7]);
            temporal::Time intervention_time =
                    temporal::Time(temporal::Duration(((intervention_frame - 1) *
                                                       causal::VariableContext::get_time_step_size())));

            if (true)
            {
                agents::FWDCarSim *fwd_car_sim = new agents::FWDCarSim(fwd_car, intervention_time);
                id_fwd_car_sim_dict.update(id, fwd_car_sim);

                env->remove_rigid_body(fwd_car);
                env->add_rigid_body(fwd_car_sim);

                map_scene_widget->insert(fwd_car_sim);

                /*
                if (agent_id != id)
                {
                    causal::VectorBufferVariable *force_variable =
                            dynamic_cast<causal::VectorBufferVariable*>(fwd_car_sim->get_other_force_variable());
                    force_variable->set_axiomatic(true);
                    causal::ScalarBufferVariable *torque_variable =
                            dynamic_cast<causal::ScalarBufferVariable*>(fwd_car_sim->get_other_torque_variable());
                    torque_variable->set_axiomatic(true);

                    for (temporal::Time current_time = intervention_time;
                         current_time <= scene->get_max_time();
                         current_time += causal::VariableContext::get_time_step_size())
                    {
                        causal::VariableContext::set_current_time(current_time);

                        geometry::Vec force;
                        fwd_car->get_other_force_variable()->get_value(force);
                        res = fwd_car_sim->get_other_force_variable()->set_value(force);
                        if (!res)
                        {
                            fwd_car_sim->get_other_force_variable()->set_value(force);
                        }

                        FP_DATA_TYPE torque;
                        fwd_car->get_other_torque_variable()->get_value(torque);
                        res = fwd_car_sim->get_other_torque_variable()->set_value(torque);
                        if (!res)
                        {
                            fwd_car_sim->get_other_torque_variable()->set_value(torque);
                        }
                    }
                }
                */
            }
        }
        else
        {
            map_scene_widget->insert(fwd_car);
        }
    }

    map_scene_widget->show();

    causal::VariableContext::set_time_step_size(scene->get_time_step_size());

    int result = app.exec();

    delete map_scene_widget;

    delete frame;

    structures::IArray<agents::FWDCarSim*> const *fwd_car_sim_array =
            id_fwd_car_sim_dict.get_values();
    for (i = 0; i < fwd_car_sim_array->count(); ++i)
    {
        delete (*fwd_car_sim_array)[i];
    }

    delete scene;

    geometry::TrigBuff::destroy_instance();

    return result;
}
