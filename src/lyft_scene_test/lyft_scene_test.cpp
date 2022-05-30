
#include <ori/simcars/agent/lyft/lyft_scene.hpp>

#include <iostream>
#include <exception>

using namespace ori::simcars;

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage: ./lyft_scene_test scene_file_path" << std::endl;
        return -1;
    }

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
}
