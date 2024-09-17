
#include <ori/simcars/structures/stl/stl_set.hpp>
#include <ori/simcars/geometry/trig_buff.hpp>
#include <ori/simcars/map/thor_magni/thor_magni_map.hpp>
#include <ori/simcars/causal/variable_context.hpp>
#include <ori/simcars/agents/ped_action_extractor.hpp>
#include <ori/simcars/agents/thor_magni/thor_magni_ped_scene.hpp>

#include <iostream>
#include <exception>

using namespace ori::simcars;

int main(int argc, char *argv[])
{
    if (argc < 6)
    {
        std::cerr << "Usage: ./thor_magni_action_extraction texture_file_path "
                     "offset_json_file_path scene_file_path goals_file_path agent_id" << std::endl;
        return -1;
    }

    geometry::TrigBuff::init_instance(360000, geometry::AngleType::RADIANS);

    std::cout << "Beginning map load" << std::endl;

    map::thor_magni::ThorMagniMap map;

    map.load(argv[1], argv[2], argv[3], argv[4]);

    std::cout << "Finished map load" << std::endl;

    std::cout << "Beginning scene load" << std::endl;

    agents::IPedScene *scene;

    scene = new agents::thor_magni::ThorMagniPedScene(argv[3]);

    std::cout << "Finished scene load" << std::endl;

    uint64_t agent_id = atoi(argv[5]);

    agents::Ped *ped = scene->get_ped(agent_id);

    agents::PedActionExtractor ped_action_extractor(&map, temporal::Duration(200),
                                                    temporal::Duration(200),
                                                    temporal::Duration(200));

    structures::IArray<agents::TimePedActionPair> *ped_actions =
            ped_action_extractor.extract_actions(ped);

    for (size_t i = 0; i < ped_actions->count(); ++i)
    {
        agents::TimePedActionPair time_action_pair = (*ped_actions)[i];
        agents::Goal<uint64_t> node_goal = time_action_pair.second.node_goal;
        std::cout << "Action [" << std::to_string(i) << "]: " <<
                     "Start = " << std::to_string(time_action_pair.first.time_since_epoch().count() / 1000) << " s, " <<
                     "Node = " << std::to_string(node_goal.val) << " @ " <<
                     std::to_string(node_goal.time.time_since_epoch().count() / 1000) << " s" << std::endl;
    }

    delete ped_actions;

    delete scene;

    geometry::TrigBuff::destroy_instance();
}
