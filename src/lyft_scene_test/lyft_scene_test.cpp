
#include <ori/simcars/geometry/trig_buff.hpp>
#include <ori/simcars/agent/lyft/lyft_scene.hpp>

#include <iostream>
#include <exception>

using namespace ori::simcars;

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage: ./lyft_scene_test scene_file_path" << std::endl;
        return -1;
    }

    geometry::TrigBuff::init_instance(360000, geometry::AngleType::RADIANS);

    std::cout << "Beginning scene load" << std::endl;

    agent::IScene *scene;

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

    delete scene;

    geometry::TrigBuff::destroy_instance();
}
