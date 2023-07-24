
#include <ori/simcars/map/highd/highd_map.hpp>

#include <iostream>
#include <exception>

using namespace ori::simcars;

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage: ./highd_map_test recording_meta_file_path" << std::endl;
        return -1;
    }

    std::cout << "Beginning map load" << std::endl;

    map::highd::HighDMap map;

    try
    {
        map.load(argv[1]);
    }
    catch (std::exception const &e)
    {
        std::cerr << "Exception occured during map load:" << std::endl << e.what() << std::endl;
        return -1;
    }

    std::cout << "Finished map load" << std::endl;
}
