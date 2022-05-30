
#include <ori/simcars/geometry/trig_buff.hpp>
#include <ori/simcars/agent/lyft/lyft_scene.hpp>
#include <ori/simcars/visualisation/qscene_widget.hpp>

#include <QApplication>
#include <QFrame>

#include <iostream>
#include <exception>
#include <memory>

using namespace ori::simcars;

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage: ./qscene_widget_test scene_file_path" << std::endl;
        return -1;
    }

    std::shared_ptr<const geometry::TrigBuff> trig_buff = geometry::TrigBuff::init_instance(360000, geometry::AngleType::RADIANS);

    QApplication app(argc, argv);

    std::cout << "Beginning scene load" << std::endl;

    std::shared_ptr<const agent::IScene> scene;

    try
    {
        scene = agent::lyft::LyftScene::load(argv[1]);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Exception occured during scene load:" << std::endl << e.what() << std::endl;
        return -1;
    }

    std::cout << "Finished scene load" << std::endl;

    std::shared_ptr<QFrame> frame(new QFrame());
    frame->setWindowTitle("QSceneWidget Test");
    frame->setFixedSize(800, 800);
    frame->show();

    std::shared_ptr<visualisation::QSceneWidget> scene_widget(
                new visualisation::QSceneWidget(
                    scene,
                    frame.get(),
                    QPoint(20, 20),
                    QSize(760, 760)));
    std::shared_ptr<structures::IArray<uint32_t>> focal_ego_agents(
                new structures::stl::STLStackArray<uint32_t>
                    { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
                      11, 12, 13, 14, 15, 16, 17, 18, 19 });
    scene_widget->set_focal_ego_agents(focal_ego_agents);
    scene_widget->show();

    return app.exec();
}
