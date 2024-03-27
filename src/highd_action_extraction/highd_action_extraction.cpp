
#include <ori/simcars/structures/stl/stl_set.hpp>
#include <ori/simcars/geometry/trig_buff.hpp>
#include <ori/simcars/map/highd/highd_map.hpp>
#include <ori/simcars/causal/variable_context.hpp>
#include <ori/simcars/agents/fwd_car_action_extractor.hpp>
#include <ori/simcars/agents/highd/highd_fwd_car_scene.hpp>

#include <iostream>
#include <exception>

using namespace ori::simcars;

int main(int argc, char *argv[])
{
    if (argc < 6)
    {
        std::cerr << "Usage: ./highd_action_extraction recording_meta_file_path tracks_meta_file_path "
                     "tracks_file_path start_frame end_frame [agent_id]" <<
                     std::endl;
        return -1;
    }

    size_t start_frame = atoi(argv[4]);
    size_t end_frame = atoi(argv[5]);

    geometry::TrigBuff::init_instance(360000, geometry::AngleType::RADIANS);

    std::cout << "Beginning map load" << std::endl;

    map::highd::HighDMap map;

    map.load(argv[1]);

    std::cout << "Finished map load" << std::endl;

    std::cout << "Beginning scene load" << std::endl;

    agents::IFWDCarScene *scene;

    scene = new agents::highd::HighDFWDCarScene(argv[2], argv[3], start_frame, end_frame);

    std::cout << "Finished scene load" << std::endl;

    uint64_t agent_id;
    if (argc > 6)
    {
        agent_id = atoi(argv[6]);
    }

    agents::FWDCar *fwd_car = scene->get_fwd_car(agent_id);

    agents::FWDCarActionExtractor fwd_car_action_extractor(&map, temporal::Duration(500),
                                                           temporal::Duration(2500), 0.1,
                                                           1.0, temporal::Duration(500));

    structures::IArray<agents::TimeFWDCarActionPair> *fwd_car_actions =
            fwd_car_action_extractor.extract_actions(fwd_car);

    for (size_t i = 0; i < fwd_car_actions->count(); ++i)
    {
        agents::TimeFWDCarActionPair time_action_pair = (*fwd_car_actions)[i];
        agents::Goal<FP_DATA_TYPE> speed_goal = time_action_pair.second.speed_goal;
        agents::Goal<uint64_t> lane_goal = time_action_pair.second.lane_goal;
        std::cout << "Action [" << std::to_string(i) << "]: " <<
                     "Start = " << std::to_string(time_action_pair.first.time_since_epoch().count() / 1000) << " s, " <<
                     "Speed = " << std::to_string(speed_goal.val) << " m/s @ " <<
                     std::to_string(speed_goal.time.time_since_epoch().count() / 1000) << " s, " <<
                     "Lane = " << std::to_string(lane_goal.val) << " @ " <<
                     std::to_string(lane_goal.time.time_since_epoch().count() / 1000) << " s" << std::endl;
    }

    delete fwd_car_actions;

    delete scene;

    geometry::TrigBuff::destroy_instance();
}
