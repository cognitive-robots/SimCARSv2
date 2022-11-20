
#include <ori/simcars/geometry/trig_buff.hpp>
#include <ori/simcars/agent/lyft/lyft_scene.hpp>
#include <ori/simcars/visualisation/qscene_widget.hpp>

#include <QApplication>
#include <QFrame>

#include <iostream>
#include <exception>

using namespace ori::simcars;

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage: ./qscene_widget_test scene_file_path" << std::endl;
        return -1;
    }

    geometry::TrigBuff const *trig_buff = geometry::TrigBuff::init_instance(360000, geometry::AngleType::RADIANS);

    QApplication app(argc, argv);

    std::cout << "Beginning scene load" << std::endl;

    agent::IScene const *scene;

    try
    {
        scene = agent::lyft::LyftScene::load(argv[1]);
    }
    catch (std::exception const &e)
    {
        std::cerr << "Exception occured during scene load:" << std::endl << e.what() << std::endl;
        return -1;
    }

    std::cout << "Finished scene load" << std::endl;

    structures::IStackArray<std::string> *focal_entities = new structures::stl::STLStackArray<std::string>();

    structures::IArray<agent::IEntity const*> *entities = scene->get_entities();

    for (size_t i = 0; i < entities->count(); ++i)
    {
        agent::IEntity const *entity = (*entities)[i];
        try
        {
            agent::IValuelessConstant const *ego_valueless_constant =
                    entity->get_constant_parameter(entity->get_name() + ".ego");

            agent::IConstant<bool> const *ego_constant = dynamic_cast<agent::IConstant<bool> const*>(ego_valueless_constant);

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

    delete entities;

    QFrame *frame = new QFrame();
    frame->setWindowTitle("QSceneWidget Test");
    frame->setFixedSize(800, 800);
    frame->show();

    visualisation::QSceneWidget *scene_widget =
            new visualisation::QSceneWidget(
                scene,
                frame,
                QPoint(20, 20),
                QSize(760, 760));
    scene_widget->set_focal_entities(focal_entities);
    scene_widget->show();

    int result = app.exec();

    delete scene_widget;

    delete frame;

    delete scene;

    geometry::TrigBuff::destroy_instance();

    return result;
}
