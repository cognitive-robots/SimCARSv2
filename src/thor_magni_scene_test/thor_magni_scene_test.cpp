
#include <ori/simcars/agents/thor_magni/thor_magni_ped_scene.hpp>

#include <exception>

using namespace ori::simcars;

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage: ./thor_magni_scene_test scene_file_path" << std::endl;
        return -1;
    }

    std::cout << "Beginning scene load" << std::endl;

    agents::IPedScene const *scene;

    scene = new agents::thor_magni::ThorMagniPedScene(argv[1]);

    std::cout << "Finished scene load" << std::endl;

    delete scene;
}
