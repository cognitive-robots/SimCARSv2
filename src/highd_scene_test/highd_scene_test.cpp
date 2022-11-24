
#include <ori/simcars/structures/stl/stl_set.hpp>
#include <ori/simcars/geometry/trig_buff.hpp>
#include <ori/simcars/agent/highd/highd_scene.hpp>

#include <iostream>
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

    structures::ISet<std::string> *agent_names = new structures::stl::STLSet<std::string>(
                {
                    "non_ego_vehicle_1",
                    "non_ego_vehicle_2",
                    "non_ego_vehicle_3",
                    "non_ego_vehicle_4",
                    "non_ego_vehicle_5",
                    "non_ego_vehicle_6",
                    "non_ego_vehicle_7",
                    "non_ego_vehicle_8",
                    "non_ego_vehicle_9"
                });

    agent::IScene const *scene;

    try
    {
        scene = agent::highd::HighDScene::load(argv[1], argv[2], agent_names);
    }
    catch (std::exception const &e)
    {
        std::cerr << "Exception occured during scene load:" << std::endl << e.what() << std::endl;
        return -1;
    }

    std::cout << "Finished scene load" << std::endl;

    delete agent_names;

    delete scene;

    geometry::TrigBuff::destroy_instance();
}
