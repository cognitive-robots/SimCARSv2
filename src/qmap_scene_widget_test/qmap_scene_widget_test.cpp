
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
        std::cerr << "Usage: ./qmap_map_scene_widget_test map_file_path scene_file_path" << std::endl;
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

    std::shared_ptr<QFrame> frame(new QFrame());
    frame->setWindowTitle("QSceneWidget Test");
    frame->setFixedSize(1000, 1000);
    frame->show();

    std::shared_ptr<visualisation::QMapSceneWidget<std::string>> map_scene_widget(
                new visualisation::QMapSceneWidget<std::string>(
                    map,
                    scene,
                    frame.get(),
                    QPoint(20, 20),
                    QSize(960, 960)));
    std::shared_ptr<const structures::stl::STLStackArray<std::shared_ptr<const agent::IAgent>>> ego_agents =
            scene->get_ego_agents();
    std::shared_ptr<structures::IArray<uint32_t>> focal_agent_ids(
                new structures::stl::STLStackArray<uint32_t>(ego_agents->count()));
    scene->get_ego_agents()->map_to<uint32_t>(
                [](const std::shared_ptr<const agent::IAgent> &agent) { return agent->get_id(); },
                *focal_agent_ids);
    map_scene_widget->set_focal_ego_agents(focal_agent_ids);
    map_scene_widget->show();

    return app.exec();
}
