
#include <ori/simcars/geometry/trig_buff.hpp>
#include <ori/simcars/map/plg/plg_map.hpp>

#include <iostream>
#include <exception>

using namespace ori::simcars;

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage: ./plg_map_test plg_file_path" << std::endl;
        return -1;
    }

    geometry::TrigBuff::init_instance(360000, geometry::AngleType::RADIANS);

    std::cout << "Beginning map load" << std::endl;

    map::plg::PLGMap map;

    map.load(argv[1]);

    std::cout << "Finished map load" << std::endl;

    geometry::TrigBuff::destroy_instance();
}
