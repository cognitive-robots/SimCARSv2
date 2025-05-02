
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
    if (argc < 5 || (argc > 5 && argc < 7))
    {
        std::cerr << "Usage: ./thor_magni_visualisation texture_file_path offset_json_file_path "
                     "scene_file_path goals_file_path [start_frame end_frame] "
                     "[causing_agent_id] [affected_agent_id]" << std::endl;
        return -1;
    }

    QApplication app(argc, argv);

    std::cout << "Beginning map load" << std::endl;

    map::thor_magni::ThorMagniMap map;

    map.load(argv[1], argv[2], argv[3], argv[4]);

    std::cout << "Finished map load" << std::endl;

    std::cout << "Beginning scene load" << std::endl;

    agents::IPedScene *scene;

    if (argc > 5)
    {
        size_t start_frame = atoi(argv[5]);
        size_t end_frame = atoi(argv[6]);
        scene = new agents::thor_magni::ThorMagniPedScene(argv[3], start_frame, end_frame);
    }
    else
    {
        scene = new agents::thor_magni::ThorMagniPedScene(argv[3]);
    }

    structures::IArray<agents::Ped*> const *agents = scene->get_peds();

    std::cout << "Finished scene load" << std::endl;

    uint64_t causing_agent_id;
    if (argc > 7)
    {
        causing_agent_id = std::atoll(argv[7]);
    }
    uint64_t affected_agent_id;
    if (argc > 8)
    {
        affected_agent_id = std::atoll(argv[8]);
    }

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
                std::chrono::milliseconds(40), 1.0, 1260.0 / map.get_max_dim_size(),
                visualisation::QPedMapAgentsWidget::FocusMode::FIXED);

    map_scene_widget->set_focal_position(map.get_map_centre());

    if (argc > 7)
    {
        map_scene_widget->set_agent_colour(causing_agent_id, sf::Color::Cyan);
    }
    if (argc > 8)
    {
        map_scene_widget->set_agent_colour(affected_agent_id, sf::Color::Magenta);
    }

    for (size_t i = 0; i < agents->count(); ++i)
    {
        agents::Ped *agent = (*agents)[i];
        map_scene_widget->insert(agent);
    }

    map_scene_widget->show();

    int result = app.exec();

    delete map_scene_widget;

    delete frame;

    delete scene;

    return result;
}
