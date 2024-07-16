
#include <ori/simcars/structures/stl/stl_set.hpp>
#include <ori/simcars/geometry/trig_buff.hpp>
#include <ori/simcars/map/highd/highd_map.hpp>
#include <ori/simcars/causal/variable_context.hpp>
#include <ori/simcars/agents/thor_magni/thor_magni_ped_scene.hpp>
#include <ori/simcars/visualisation/qped_agents_widget.hpp>

#include <QApplication>
#include <QFrame>

#include <iostream>
#include <exception>

using namespace ori::simcars;

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage: ./thor_magni_visualisation scene_file_path" <<
                     std::endl;
        return -1;
    }

    QApplication app(argc, argv);

    std::cout << "Beginning scene load" << std::endl;

    agents::IPedScene *scene;

    scene = new agents::thor_magni::ThorMagniPedScene(argv[1]);

    structures::IArray<agents::PointMass*> const *agents = scene->get_point_masses();

    std::cout << "Finished scene load" << std::endl;

    QFrame *frame = new QFrame();
    frame->setWindowTitle("SIMCARS Demo");
    frame->setFixedSize(1280, 1280);
    frame->show();

    visualisation::QPedAgentsWidget *map_scene_widget =
            new visualisation::QPedAgentsWidget(
                frame,
                QPoint(10, 10),
                QSize(1260, 1260),
                scene->get_min_time(),
                scene->get_max_time(),
                std::chrono::milliseconds(40), 1.0, 20.0,
                visualisation::QPedAgentsWidget::FocusMode::FIXED);

    for (size_t i = 0; i < agents->count(); ++i)
    {
        agents::PointMass *agent = (*agents)[i];
        map_scene_widget->insert(agent);
    }

    map_scene_widget->show();

    int result = app.exec();

    delete map_scene_widget;

    delete frame;

    delete scene;

    return result;
}
