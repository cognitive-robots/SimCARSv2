#include <ori/simcars/map/lyft/lyft_map.hpp>

#include <iostream>
#include <exception>

using namespace ori::simcars;

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage: ./lyft_map_test map_file_path" << std::endl;
        return -1;
    }

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
}
