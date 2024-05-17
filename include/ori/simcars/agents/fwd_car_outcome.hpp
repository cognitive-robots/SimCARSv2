#pragma once

#include <ori/simcars/geometry/defines.hpp>

#include <cstdint>
#include <cmath>
#include <iostream>

namespace ori
{
namespace simcars
{
namespace agents
{

struct FWDCarOutcome
{
    FP_DATA_TYPE lane_transitions;
    FP_DATA_TYPE final_speed;
    FP_DATA_TYPE dist_headway;
    FP_DATA_TYPE max_env_force_mag;
    bool action_done;

    friend bool operator ==(FWDCarOutcome const &outcome_1, FWDCarOutcome const &outcome_2)
    {
        return outcome_1.lane_transitions == outcome_2.lane_transitions &&
                outcome_1.final_speed == outcome_2.final_speed &&
                outcome_1.dist_headway == outcome_2.dist_headway &&
                outcome_1.max_env_force_mag == outcome_2.max_env_force_mag &&
                outcome_1.action_done == outcome_2.action_done;
    }

    friend FP_DATA_TYPE diff(FWDCarOutcome const &outcome_1, FWDCarOutcome const &outcome_2,
                             FP_DATA_TYPE scale = 0.1,
                             FP_DATA_TYPE lane_transitions_scale = 100.0,
                             FP_DATA_TYPE final_speed_scale = 1.0,
                             FP_DATA_TYPE dist_headway_scale = 0.1,
                             FP_DATA_TYPE max_env_force_mag_scale = 0.01,
                             FP_DATA_TYPE action_done_scale = 100.0)
    {
        FP_DATA_TYPE lane_transitions_diff = std::pow(outcome_1.lane_transitions -
                                                      outcome_2.lane_transitions, 2.0);
        FP_DATA_TYPE final_speed_diff = std::pow((outcome_1.final_speed - outcome_2.final_speed) /
                                                 (0.5 * (outcome_1.final_speed + outcome_2.final_speed)),
                                                 2.0);
        FP_DATA_TYPE dist_headway_diff = std::pow(outcome_1.dist_headway / outcome_1.final_speed -
                                                  outcome_2.dist_headway / outcome_2.final_speed,
                                                  2.0);
        FP_DATA_TYPE max_env_force_mag_diff = std::pow(outcome_1.max_env_force_mag -
                                                       outcome_2.max_env_force_mag, 2.0);
        FP_DATA_TYPE action_done_diff = std::pow(outcome_1.action_done -
                                                 outcome_2.action_done, 2.0);
        return scale * std::sqrt(lane_transitions_scale * lane_transitions_diff +
                                 final_speed_scale * final_speed_diff +
                                 dist_headway_scale * dist_headway_diff +
                                 max_env_force_mag_scale * max_env_force_mag_diff +
                                 action_done_scale * action_done_diff);
    }

    friend std::ostream& operator<<(std::ostream& output_stream, const FWDCarOutcome& outcome)
    {
        return output_stream << "Lane Trans. = " << outcome.lane_transitions << ", " <<
                                "Final Speed = " << outcome.final_speed << " m/s, " <<
                                "Dist. Headway = " << outcome.dist_headway << " m, " <<
                                "Max. Env. Force Mag. = " << outcome.max_env_force_mag << " N, " <<
                                "Action Done = " << outcome.action_done;
    }
};

}
}
}
