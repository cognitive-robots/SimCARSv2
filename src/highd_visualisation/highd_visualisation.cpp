
#include <ori/simcars/structures/stl/stl_set.hpp>
#include <ori/simcars/geometry/trig_buff.hpp>
#include <ori/simcars/map/highd/highd_map.hpp>
#include <ori/simcars/causal/variable_context.hpp>
#include <ori/simcars/agents/fwd_car_sim.hpp>
#include <ori/simcars/agents/highd/highd_fwd_car_scene.hpp>
#include <ori/simcars/visualisation/qmap_agents_widget.hpp>

#include <QApplication>
#include <QFrame>

#include <iostream>
#include <exception>

using namespace ori::simcars;

int main(int argc, char *argv[])
{
    if (argc < 6)
    {
        std::cerr << "Usage: ./highd_visualisation recording_meta_file_path tracks_meta_file_path "
                     "tracks_file_path start_frame end_frame [agent_id]" <<
                     std::endl;
        return -1;
    }

    size_t start_frame = atoi(argv[4]);
    size_t end_frame = atoi(argv[5]);

    geometry::TrigBuff::init_instance(360000, geometry::AngleType::RADIANS);

    QApplication app(argc, argv);

    std::cout << "Beginning map load" << std::endl;

    map::highd::HighDMap map;

    map.load(argv[1]);

    std::cout << "Finished map load" << std::endl;

    std::cout << "Beginning scene load" << std::endl;

    agents::IFWDCarScene *scene;

    scene = new agents::highd::HighDFWDCarScene(argv[2], argv[3], start_frame, end_frame);

    structures::IArray<agents::FWDCar*> const *fwd_cars = scene->get_fwd_cars();

    std::cout << "Finished scene load" << std::endl;

    QFrame *frame = new QFrame();
    frame->setWindowTitle("SIMCARS Demo");
    frame->setFixedSize(1280, 1280);
    frame->show();

    geometry::Vec focal_position(210.0f, 19.0f);

    visualisation::QMapAgentsWidget *map_scene_widget =
            new visualisation::QMapAgentsWidget(
                &map,
                frame,
                QPoint(10, 10),
                QSize(1260, 1260),
                scene->get_min_time(),
                scene->get_max_time(),
                std::chrono::milliseconds(40), 1.0, 3.0,
                visualisation::QMapAgentsWidget::FocusMode::FIXED);

    map_scene_widget->set_focal_position(focal_position);

    uint64_t agent_id;
    if (argc > 6)
    {
        agent_id = atoi(argv[6]);
        map_scene_widget->set_agent_colour(agent_id, sf::Color::Cyan);
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
