
#include <ori/simcars/agents/highd/highd_fwd_car_scene.hpp>

#include <exception>

using namespace ori::simcars;

int main(int argc, char *argv[])
{
    if (argc < 3)
    {
        std::cerr << "Usage: ./highd_scene_test tracks_meta_file_path tracks_file_path" << std::endl;
        return -1;
    }

    geometry::TrigBuff::init_instance(360000, geometry::AngleType::RADIANS);

    std::cout << "Beginning scene load" << std::endl;

    agents::IFWDCarScene const *scene;

    scene = new agents::highd::HighDFWDCarScene(argv[1], argv[2]);

    std::cout << "Finished scene load" << std::endl;

    delete scene;

    geometry::TrigBuff::destroy_instance();
}
