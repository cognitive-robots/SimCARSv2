
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

    std::shared_ptr<structures::IStackArray<std::string>> focal_entities(
                new structures::stl::STLStackArray<std::string>());

    std::shared_ptr<structures::IArray<std::shared_ptr<const agent::IEntity>>> entities =
            scene->get_entities();

    for (size_t i = 0; i < entities->count(); ++i)
    {
        std::shared_ptr<const agent::IEntity> entity = (*entities)[i];
        try
        {
            std::shared_ptr<const agent::IValuelessConstant> ego_valueless_constant =
                    entity->get_constant_parameter(entity->get_name() + ".ego");

            std::shared_ptr<const agent::IConstant<bool>> ego_constant =
                    std::static_pointer_cast<const agent::IConstant<bool>>(ego_valueless_constant);

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
    map_scene_widget->set_focal_entities(focal_entities);
    map_scene_widget->show();

    return app.exec();
}
