
#include <ori/simcars/agents/otherd/otherd_fwd_car_scene.hpp>

#include <exception>

using namespace ori::simcars;

int main(int argc, char *argv[])
{
    if (argc < 4)
    {
        std::cerr << "Usage: ./otherd_scene_test recording_meta_file_path tracks_meta_file_path tracks_file_path" << std::endl;
        return -1;
    }

    geometry::TrigBuff::init_instance(360000, geometry::AngleType::RADIANS);

    std::cout << "Beginning scene load" << std::endl;

    agents::IFWDCarScene const *scene;

    scene = new agents::otherd::OtherDFWDCarScene(argv[1], argv[2], argv[3]);

    std::cout << "Finished scene load" << std::endl;

    delete scene;

    geometry::TrigBuff::destroy_instance();
}
