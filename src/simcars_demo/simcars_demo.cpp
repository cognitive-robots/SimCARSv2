
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
#include <memory>

using namespace ori::simcars;

int main(int argc, char* argv[])
{
    if (argc < 3)
    {
        std::cerr << "Usage: ./simcars_demo map_file_path scene_file_path" << std::endl;
        return -1;
    }

    std::shared_ptr<const geometry::TrigBuff> trig_buff = geometry::TrigBuff::init_instance(360000, geometry::AngleType::RADIANS);

    QApplication app(argc, argv);

    std::cout << "Beginning map load" << std::endl;

    std::shared_ptr<const map::IMap<std::string>> map;

    try
    {
        map = map::lyft::LyftMap::load(argv[1]);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Exception occured during map load:" << std::endl << e.what() << std::endl;
        return -1;
    }

    std::cout << "Finished map load" << std::endl;

    std::cout << "Beginning scene load" << std::endl;

    std::shared_ptr<const agent::IDrivingScene> scene;

    try
    {
        scene = agent::lyft::LyftScene::load(argv[2]);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Exception occured during scene load:" << std::endl << e.what() << std::endl;
        return -1;
    }

    std::cout << "Finished scene load" << std::endl;

    std::cout << "Beginning action extraction" << std::endl;

    std::shared_ptr<const agent::IDrivingScene> scene_with_actions;

    try
    {
        scene_with_actions = agent::DrivingGoalExtractionScene::construct_from(scene, map);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Exception occured during action extraction:" << std::endl << e.what() << std::endl;
        return -1;
    }

    std::cout << "Finished action extraction" << std::endl;

    std::shared_ptr<const structures::IArray<std::shared_ptr<const agent::IEntity>>> entities = scene->get_entities();

    if (entities->count() == 0)
    {
        std::cerr << "No agents were present in the scene" << std::endl;
        return -1;
    }

    bool ego_focal_agents = false;
    std::shared_ptr<structures::IArray<std::string>> focal_agent_ids(new structures::stl::STLStackArray<std::string>(1));

    size_t i;

    for (i = 0; i < entities->count(); ++i)
    {
        std::shared_ptr<const agent::IDrivingAgent> driving_agent =
                std::dynamic_pointer_cast<const agent::IDrivingAgent>((*entities)[i]);

        if (driving_agent->get_ego_constant()->get_value())
        {
            ego_focal_agents = true;
            (*focal_agent_ids)[0] = driving_agent->get_name();
            break;
        }
    }

    if (!ego_focal_agents)
    {
        std::shared_ptr<const agent::IDrivingAgent> driving_agent =
                std::dynamic_pointer_cast<const agent::IDrivingAgent>((*entities)[0]);
        (*focal_agent_ids)[0] = driving_agent->get_name();
    }

    temporal::Time scene_half_way_timestamp = scene->get_min_temporal_limit() +
            (scene->get_max_temporal_limit() - scene->get_min_temporal_limit()) / 2;

    temporal::Duration time_step(100);

    std::shared_ptr<agent::IDrivingAgentController> driving_agent_controller(
                new agent::BasicDrivingAgentController(map, time_step, 10));

    std::shared_ptr<agent::IDrivingSimulator> driving_simulator(
                new agent::BasicDrivingSimulator(driving_agent_controller));

    std::shared_ptr<const agent::IDrivingScene> simulated_scene =
            agent::DrivingSimulationScene::construct_from(
                scene_with_actions, driving_simulator, time_step, scene_half_way_timestamp);

    std::shared_ptr<QFrame> frame(new QFrame());
    frame->setWindowTitle("SIMCARS Demo");
    frame->setFixedSize(1600, 800);
    frame->show();

    /*
    std::shared_ptr<visualisation::QMapSceneWidget<std::string>> map_scene_widget(
                new visualisation::QMapSceneWidget<std::string>(
                    map,
                    scene,
                    frame.get(),
                    QPoint(20, 20),
                    QSize(760, 760),
                    1.0f,
                    10.0f,
                    1.0f));
    map_scene_widget->set_focal_entities(focal_agent_ids);
    map_scene_widget->show();
    */

    std::shared_ptr<visualisation::QMapSceneWidget<std::string>> map_simulated_scene_widget(
                new visualisation::QMapSceneWidget<std::string>(
                    map,
                    simulated_scene,
                    frame.get(),
                    QPoint(820, 20),
                    QSize(760, 760),
                    1.0f,
                    30.0f,
                    1.0f));
    map_simulated_scene_widget->set_focal_entities(focal_agent_ids);
    map_simulated_scene_widget->show();

    return app.exec();
}
