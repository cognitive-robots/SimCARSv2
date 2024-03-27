#pragma once

#include <ori/simcars/structures/dictionary_interface.hpp>
#include <ori/simcars/map/map_interface.hpp>
#include <ori/simcars/agents/typedefs.hpp>
#include <ori/simcars/agents/fwd_car_action.hpp>
#include <ori/simcars/agents/fwd_car.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class FWDCarActionExtractor
{
    map::IMap *map;
    temporal::Duration resolution;
    temporal::Duration action_min_duration;
    FP_DATA_TYPE action_min_lon_lin_acc;
    FP_DATA_TYPE action_min_lon_lin_vel_diff;
    temporal::Duration lane_min_duration;

    structures::IArray<TimeGoalPair<FP_DATA_TYPE>>* extract_speed_goals(
            agents::FWDCar *fwd_car) const;
    structures::IArray<TimeGoalPair<uint64_t>>* extract_lane_goals(agents::FWDCar *fwd_car) const;

public:
    FWDCarActionExtractor(map::IMap *map, temporal::Duration resolution,
                          temporal::Duration action_min_duration,
                          FP_DATA_TYPE action_min_lon_lin_acc,
                          FP_DATA_TYPE action_min_lon_lin_vel_diff,
                          temporal::Duration lane_min_duration);

    ~FWDCarActionExtractor() = default;

    structures::IArray<TimeFWDCarActionPair>* extract_actions(agents::FWDCar *fwd_car) const;
};

}
}
}
