
#include <ori/simcars/structures/stl/stl_set.hpp>
#include <ori/simcars/geometry/trig_buff.hpp>
#include <ori/simcars/map/highd/highd_map.hpp>
#include <ori/simcars/agent/driving_goal_extraction_scene.hpp>
#include <ori/simcars/agent/basic_driving_agent_controller.hpp>
#include <ori/simcars/agent/basic_driving_simulator.hpp>
#include <ori/simcars/agent/driving_simulation_scene.hpp>
#include <ori/simcars/agent/highd/highd_scene.hpp>
#include <ori/simcars/visualisation/qmap_scene_widget.hpp>

#include <QApplication>
#include <QFrame>

#include <iostream>
#include <exception>

using namespace ori::simcars;

int main(int argc, char *argv[])
{
    if (argc < 4)
    {
        std::cerr << "Usage: ./highd_simcars_demo recording_meta_file_path tracks_meta_file_path tracks_file_path" << std::endl;
        return -1;
    }

    geometry::TrigBuff const *trig_buff = geometry::TrigBuff::init_instance(360000, geometry::AngleType::RADIANS);

    QApplication app(argc, argv);

    std::cout << "Beginning map load" << std::endl;

    map::IMap<uint8_t> const *map;

    try
    {
        map = map::highd::HighDMap::load(argv[1]);
    }
    catch (std::exception const &e)
    {
        std::cerr << "Exception occured during map load:" << std::endl << e.what() << std::endl;
        return -1;
    }

    std::cout << "Finished map load" << std::endl;

    std::cout << "Beginning scene load" << std::endl;

    structures::ISet<std::string> *agent_names = new structures::stl::STLSet<std::string>(
                {
                    "non_ego_vehicle_1",
                    "non_ego_vehicle_2",
                    "non_ego_vehicle_3",
                    "non_ego_vehicle_4",
                    "non_ego_vehicle_5",
                    "non_ego_vehicle_6",
                    "non_ego_vehicle_7",
                    "non_ego_vehicle_8",
                    "non_ego_vehicle_9"
                });

    agent::IDrivingScene const *scene;

    try
    {
        scene = agent::highd::HighDScene::load(argv[2], argv[3], agent_names);
    }
    catch (std::exception const &e)
    {
        std::cerr << "Exception occured during scene load:" << std::endl << e.what() << std::endl;
        return -1;
    }

    std::cout << "Finished scene load" << std::endl;

    delete agent_names;

    std::cout << "Beginning action extraction" << std::endl;

    agent::IDrivingScene const *scene_with_actions;

    try
    {
        scene_with_actions = agent::DrivingGoalExtractionScene::construct_from(scene, map);
    }
    catch (std::exception const &e)
    {
        std::cerr << "Exception occured during action extraction:" << std::endl << e.what() << std::endl;
        return -1;
    }

    std::cout << "Finished action extraction" << std::endl;

    structures::IStackArray<std::string> *focal_entities =
            new structures::stl::STLStackArray<std::string>();

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

    temporal::Duration time_step(40);

    agent::IDrivingAgentController *driving_agent_controller =
                new agent::BasicDrivingAgentController(map, time_step, 10);

    agent::IDrivingSimulator *driving_simulator =
                new agent::BasicDrivingSimulator(driving_agent_controller);

    agent::IDrivingScene const *simulated_scene =
            agent::DrivingSimulationScene::construct_from(
                scene_with_actions, driving_simulator, time_step, scene_half_way_timestamp);

    QFrame *frame = new QFrame();
    frame->setWindowTitle("SIMCARS Demo");
    frame->setFixedSize(1600, 800);
    frame->show();

    visualisation::QMapSceneWidget<uint8_t> *map_scene_widget =
                new visualisation::QMapSceneWidget<uint8_t>(
                    map,
                    scene,
                    frame,
                    QPoint(20, 20),
                    QSize(760, 760),
                    1.0f,
                    10.0f,
                    1.0f);
    map_scene_widget->set_focal_entities(focal_entities);
    map_scene_widget->show();

    visualisation::QMapSceneWidget<uint8_t> *map_simulated_scene_widget =
                new visualisation::QMapSceneWidget<uint8_t>(
                    map,
                    simulated_scene,
                    frame,
                    QPoint(820, 20),
                    QSize(760, 760),
                    1.0f,
                    30.0f,
                    1.0f);
    map_simulated_scene_widget->set_focal_entities(
                new structures::stl::STLStackArray(focal_entities));
    map_simulated_scene_widget->show();

    int result = app.exec();

    delete map_simulated_scene_widget;
    delete map_scene_widget;

    delete frame;

    delete simulated_scene;
    delete scene_with_actions;
    delete scene;

    delete map;

    geometry::TrigBuff::destroy_instance();

    return result;
}
