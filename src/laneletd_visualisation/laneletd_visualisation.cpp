
#include <ori/simcars/structures/stl/stl_set.hpp>
#include <ori/simcars/geometry/trig_buff.hpp>
#include <ori/simcars/map/laneletd/laneletd_map.hpp>
#include <ori/simcars/causal/variable_context.hpp>
#include <ori/simcars/agents/fwd_car_sim.hpp>
#include <ori/simcars/agents/otherd/otherd_fwd_car_scene.hpp>
#include <ori/simcars/visualisation/qmap_agents_widget.hpp>

#include <QApplication>
#include <QFrame>

#include <iostream>
#include <exception>

using namespace ori::simcars;

int main(int argc, char *argv[])
{
    if (argc < 7)
    {
        std::cerr << "Usage: ./laneletd_visualisation lanelet_map_file_path recording_meta_file_path "
                     "tracks_meta_file_path tracks_file_path start_frame end_frame [agent_id_1] "
                     "[agent_id_2]" <<
                     std::endl;
        return -1;
    }

    size_t start_frame = atoi(argv[5]);
    size_t end_frame = atoi(argv[6]);

    geometry::TrigBuff::init_instance(360000, geometry::AngleType::RADIANS);

    QApplication app(argc, argv);

    std::cout << "Beginning map load" << std::endl;

    map::laneletd::LaneletDMap map;

    map.load(argv[1], argv[2]);

    std::cout << "Finished map load" << std::endl;

    std::cout << "Beginning scene load" << std::endl;

    agents::IFWDCarScene *scene;

    scene = new agents::otherd::OtherDFWDCarScene(argv[2], argv[3], argv[4], start_frame, end_frame);

    structures::IArray<agents::FWDCar*> const *fwd_cars = scene->get_fwd_cars();

    std::cout << "Finished scene load" << std::endl;

    QFrame *frame = new QFrame();
    frame->setWindowTitle("SIMCARS Demo");
    frame->setFixedSize(1280, 1280);
    frame->show();

    visualisation::QMapAgentsWidget *map_scene_widget =
            new visualisation::QMapAgentsWidget(
                &map,
                frame,
                QPoint(10, 10),
                QSize(1260, 1260),
                scene->get_min_time(),
                scene->get_max_time(),
                std::chrono::milliseconds(40), 1.0, 1260.0 / map.get_max_dim_size(),
                visualisation::QMapAgentsWidget::FocusMode::FIXED);

    map_scene_widget->set_focal_position(map.get_map_centre());

    if (argc > 7)
    {
        uint64_t agent_id_1 = atoi(argv[7]);
        map_scene_widget->set_agent_colour(agent_id_1, sf::Color::Cyan);
    }

    if (argc > 8)
    {
        uint64_t agent_id_2 = atoi(argv[8]);
        map_scene_widget->set_agent_colour(agent_id_2, sf::Color::Magenta);
    }

    for (size_t i = 9; i < argc; ++i)
    {
        uint64_t other_rel_agent_id = std::atoll(argv[i]);
        map_scene_widget->set_agent_colour(other_rel_agent_id, sf::Color::Yellow);
    }

    for (size_t i = 0; i < fwd_cars->count(); ++i)
    {
        agents::FWDCar *fwd_car = (*fwd_cars)[i];
        map_scene_widget->insert(fwd_car);
    }

    map_scene_widget->show();

    int result = app.exec();

    delete map_scene_widget;

    delete frame;

    delete scene;

    geometry::TrigBuff::destroy_instance();

    return result;
}
