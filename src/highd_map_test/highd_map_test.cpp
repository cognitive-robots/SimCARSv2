
#include <ori/simcars/map/highd/highd_map.hpp>

#include <iostream>
#include <exception>

using namespace ori::simcars;

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage: ./highd_map_test map_file_path" << std::endl;
        return -1;
    }

    std::cout << "Beginning map load" << std::endl;

    map::IMap<uint8_t> const *map;

    try
    {
        map = map::highd::HighDMap::load(argv[1]);
    }
    catch (std::exception const &e)
    {
        std::cerr << "Exception occured during map load:" << std::endl << e.what() << std::endl;
        return -1;
    }

    std::cout << "Finished map load" << std::endl;

    delete map;
}
