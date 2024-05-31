
#include <ori/simcars/geometry/trig_buff.hpp>
#include <ori/simcars/map/laneletd/laneletd_map.hpp>

#include <iostream>
#include <exception>

using namespace ori::simcars;

int main(int argc, char *argv[])
{
    if (argc < 3)
    {
        std::cerr << "Usage: ./laneletd_map_test lanelet_map_file_path recording_meta_file_path" << std::endl;
        return -1;
    }

    geometry::TrigBuff::init_instance(360000, geometry::AngleType::RADIANS);

    std::cout << "Beginning map load" << std::endl;

    map::laneletd::LaneletDMap map;

    map.load(argv[1], argv[2]);

    std::cout << "Finished map load" << std::endl;

    geometry::TrigBuff::destroy_instance();
}
