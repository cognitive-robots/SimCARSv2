
#include <ori/simcars/geometry/trig_buff.hpp>
#include <ori/simcars/map/lyft/lyft_map.hpp>
#include <ori/simcars/agent/driving_goal_extraction_scene.hpp>
#include <ori/simcars/agent/basic_driving_agent_controller.hpp>
#include <ori/simcars/agent/basic_driving_simulator.hpp>
#include <ori/simcars/agent/driving_simulation_scene.hpp>
#include <ori/simcars/agent/lyft/lyft_scene.hpp>
#include <ori/simcars/visualisation/qmap_scene_widget.hpp>

#include <QApplication>
#include <QFrame>

#include <iostream>
#include <exception>

using namespace ori::simcars;

int main(int argc, char *argv[])
{
    if (argc < 3)
    {
        std::cerr << "Usage: ./lyft_simcars_demo map_file_path scene_file_path" << std::endl;
        return -1;
    }

    geometry::TrigBuff::init_instance(360000, geometry::AngleType::RADIANS);

    QApplication app(argc, argv);

    std::cout << "Beginning map load" << std::endl;

    map::IMap<std::string> const *map;

    try
    {
        map = map::lyft::LyftMap::load(argv[1]);
    }
    catch (std::exception const &e)
    {
        std::cerr << "Exception occured during map load:" << std::endl << e.what() << std::endl;
        return -1;
    }

    std::cout << "Finished map load" << std::endl;

    std::cout << "Beginning scene load" << std::endl;

    agent::IDrivingScene *scene;

    try
    {
        scene = agent::lyft::LyftScene::load(argv[2]);
    }
    catch (std::exception const &e)
    {
        std::cerr << "Exception occured during scene load:" << std::endl << e.what() << std::endl;
        return -1;
    }

    std::cout << "Finished scene load" << std::endl;

    std::cout << "Beginning action extraction" << std::endl;

    agent::IDrivingScene *scene_with_actions;

    try
    {
        scene_with_actions = agent::DrivingGoalExtractionScene<std::string>::construct_from(scene, map);
    }
    catch (std::exception const &e)
    {
        std::cerr << "Exception occured during action extraction:" << std::endl << e.what() << std::endl;
        return -1;
    }

    std::cout << "Finished action extraction" << std::endl;

    // NOTE: Kinda redundant, maybe we should only use one type of container
    structures::IStackArray<std::string> *focal_entities =
            new structures::stl::STLStackArray<std::string>();
    structures::ISet<std::string> *agent_names =
            new structures::stl::STLSet<std::string>;

    structures::IArray<agent::IEntity const*> *entities =
            scene->get_entities();

    for (size_t i = 0; i < entities->count(); ++i)
    {
        agent::IEntity const *entity = (*entities)[i];
        try
        {
            agent::IValuelessConstant const *ego_valueless_constant =
                    entity->get_constant_parameter(entity->get_name() + ".ego");

            agent::IConstant<bool> const *ego_constant =
                    dynamic_cast<agent::IConstant<bool> const*>(ego_valueless_constant);

            if (ego_constant->get_value())
            {
                focal_entities->push_back(entity->get_name());
                agent_names->insert(entity->get_name());
            }
        }
        catch (std::out_of_range)
        {
            // Entity does not have an ego parameter
        }
    }

    delete entities;

    temporal::Time scene_half_way_timestamp = scene->get_min_temporal_limit() +
            (scene->get_max_temporal_limit() - scene->get_min_temporal_limit()) / 2;

    temporal::Duration time_step(100);

    agent::IDrivingAgentController *driving_agent_controller =
                new agent::BasicDrivingAgentController<std::string>(map, time_step, 10);

    agent::IDrivingSimulator *driving_simulator =
                new agent::BasicDrivingSimulator(driving_agent_controller);

    agent::IDrivingScene *simulated_scene =
            agent::DrivingSimulationScene::construct_from(
                scene_with_actions, driving_simulator, time_step,
                scene_half_way_timestamp, agent_names);

    delete agent_names;

    QFrame *frame = new QFrame();
    frame->setWindowTitle("SIMCARS Demo");
    frame->setFixedSize(1600, 800);
    frame->show();

    /*
    visualisation::QMapSceneWidget<std::string> *map_scene_widget =
                new visualisation::QMapSceneWidget<std::string>(
                    map,
                    scene,
                    frame,
                    QPoint(20, 20),
                    QSize(760, 760),
                    1.0f,
                    30.0f,
                    1.0f);
    map_scene_widget->set_focus_mode(visualisation::QSceneWidget::FocusMode::FOCAL_AGENTS);
    map_scene_widget->set_focal_entities(focal_entities);
    map_scene_widget->show();
    */

    visualisation::QMapSceneWidget<std::string> *map_simulated_scene_widget =
                new visualisation::QMapSceneWidget<std::string>(
                    map,
                    simulated_scene,
                    frame,
                    QPoint(820, 20),
                    QSize(760, 760),
                    1.0f,
                    30.0f,
                    1.0f);
    map_simulated_scene_widget->set_focus_mode(visualisation::QSceneWidget::FocusMode::FOCAL_AGENTS);
    map_simulated_scene_widget->set_focal_entities(
                new structures::stl::STLStackArray(focal_entities));
    map_simulated_scene_widget->show();

    int result = app.exec();

    delete map_simulated_scene_widget;
    //delete map_scene_widget;

    delete frame;

    delete simulated_scene;
    delete scene_with_actions;
    delete scene;

    delete map;

    geometry::TrigBuff::destroy_instance();

    return result;
}
