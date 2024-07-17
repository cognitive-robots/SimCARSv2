
#include <ori/simcars/structures/stl/stl_set.hpp>
#include <ori/simcars/geometry/trig_buff.hpp>
#include <ori/simcars/map/thor_magni/thor_magni_map.hpp>
#include <ori/simcars/causal/variable_context.hpp>
#include <ori/simcars/agents/thor_magni/thor_magni_ped_scene.hpp>
#include <ori/simcars/visualisation/qped_map_agents_widget.hpp>

#include <QApplication>
#include <QFrame>

#include <iostream>
#include <exception>

using namespace ori::simcars;

int main(int argc, char *argv[])
{
    if (argc < 4)
    {
        std::cerr << "Usage: ./thor_magni_visualisation scene_file_path texture_file_path "
                     "offset_json_file_path" << std::endl;
        return -1;
    }

    QApplication app(argc, argv);

    std::cout << "Beginning map load" << std::endl;

    map::thor_magni::ThorMagniMap map;

    map.load(argv[2], argv[3], argv[1]);

    std::cout << "Finished map load" << std::endl;

    std::cout << "Beginning scene load" << std::endl;

    agents::IPedScene *scene;

    scene = new agents::thor_magni::ThorMagniPedScene(argv[1]);

    structures::IArray<agents::PointMass*> const *agents = scene->get_point_masses();

    std::cout << "Finished scene load" << std::endl;

    QFrame *frame = new QFrame();
    frame->setWindowTitle("SIMCARS Demo");
    frame->setFixedSize(1280, 1280);
    frame->show();

    visualisation::QPedMapAgentsWidget *map_scene_widget =
            new visualisation::QPedMapAgentsWidget(
                &map,
                frame,
                QPoint(10, 10),
                QSize(1260, 1260),
                scene->get_min_time(),
                scene->get_max_time(),
                std::chrono::milliseconds(40), 1.0, 40.0,
                visualisation::QPedMapAgentsWidget::FocusMode::FIXED);

    map_scene_widget->set_focal_position(map.get_map_centre());

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
