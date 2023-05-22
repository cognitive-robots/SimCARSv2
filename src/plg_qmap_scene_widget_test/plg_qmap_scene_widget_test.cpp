
#include <ori/simcars/geometry/trig_buff.hpp>
#include <ori/simcars/map/plg/plg_map.hpp>
#include <ori/simcars/agent/plg/plg_scene.hpp>
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
        std::cerr << "Usage: ./plg_qmap_scene_widget_test map_file_path scene_file_path" << std::endl;
        return -1;
    }

    geometry::TrigBuff::init_instance(360000, geometry::AngleType::RADIANS);

    QApplication app(argc, argv);

    std::cout << "Beginning map load" << std::endl;

    map::IMap<uint8_t> const *map;

    try
    {
        map = map::plg::PLGMap::load(argv[1]);
    }
    catch (std::exception const &e)
    {
        std::cerr << "Exception occured during map load:" << std::endl << e.what() << std::endl;
        return -1;
    }

    std::cout << "Finished map load" << std::endl;

    std::cout << "Beginning scene load" << std::endl;

    agent::IScene *scene;

    try
    {
        scene = agent::plg::PLGScene::load(argv[2]);
    }
    catch (std::exception const &e)
    {
        std::cerr << "Exception occured during scene load:" << std::endl << e.what() << std::endl;
        return -1;
    }

    std::cout << "Finished scene load" << std::endl;

    QFrame *frame = new QFrame();
    frame->setWindowTitle("QMapSceneWidget Test");
    frame->setFixedSize(1000, 1000);
    frame->show();

    visualisation::QMapSceneWidget<uint8_t> *map_scene_widget =
            new visualisation::QMapSceneWidget<uint8_t>(
                map,
                scene,
                frame,
                QPoint(20, 20),
                QSize(960, 960),
                1.0f,
                10.0f);
    map_scene_widget->set_focus_mode(visualisation::QSceneWidget::FocusMode::ALL_AGENTS);
    map_scene_widget->show();

    int result = app.exec();

    delete map_scene_widget;

    delete frame;

    delete scene;

    delete map;

    geometry::TrigBuff::destroy_instance();

    return result;
}
