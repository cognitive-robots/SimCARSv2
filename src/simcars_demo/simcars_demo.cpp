
#include <ori/simcars/geometry/trig_buff.hpp>
#include <ori/simcars/map/lyft/lyft_map.hpp>
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

    std::shared_ptr<const agent::IScene> scene;

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

    std::shared_ptr<const structures::IArray<std::shared_ptr<const agent::IAgent>>> ego_agents = scene->get_ego_agents();
    bool ego_focal_agents = true;

    std::shared_ptr<structures::IArray<uint32_t>> focal_agent_ids(new structures::stl::STLStackArray<uint32_t>(1));
    if (ego_agents->count() > 0)
    {
        (*focal_agent_ids)[0] = (*ego_agents)[0]->get_id();
    }
    else
    {
        std::shared_ptr<const structures::IArray<std::shared_ptr<const agent::IAgent>>> non_ego_agents = scene->get_non_ego_agents();
        ego_focal_agents = false;

        if (non_ego_agents->count() > 0)
        {
            (*focal_agent_ids)[0] = (*non_ego_agents)[0]->get_id();
        }
        else
        {
            std::cerr << "No agents were present in the scene" << std::endl;
            return -1;
        }
    }

    temporal::Time scene_half_way_timestamp = scene->get_min_time() + (scene->get_max_time() - scene->get_min_time()) / 2;

    std::shared_ptr<const agent::IScene> simulated_scene =
            scene->fork_simulated_scene(ego_focal_agents, (*focal_agent_ids)[0], scene_half_way_timestamp, temporal::Duration(100));

    std::shared_ptr<QFrame> frame(new QFrame());
    frame->setWindowTitle("QSceneWidget Test");
    frame->setFixedSize(1600, 800);
    frame->show();

    std::shared_ptr<visualisation::QMapSceneWidget<std::string>> map_scene_widget(
                new visualisation::QMapSceneWidget<std::string>(
                    map,
                    scene,
                    frame.get(),
                    QPoint(20, 20),
                    QSize(760, 760),
                    1.0f,
                    30.0f,
                    1.0f));
    if (ego_focal_agents)
    {
        map_scene_widget->set_focal_ego_agents(focal_agent_ids);
    }
    else
    {
        map_scene_widget->set_focal_non_ego_agents(focal_agent_ids);
    }
    map_scene_widget->show();

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
    if (ego_focal_agents)
    {
        map_simulated_scene_widget->set_focal_ego_agents(focal_agent_ids);
    }
    else
    {
        map_simulated_scene_widget->set_focal_non_ego_agents(focal_agent_ids);
    }
    map_simulated_scene_widget->show();

    return app.exec();
}
