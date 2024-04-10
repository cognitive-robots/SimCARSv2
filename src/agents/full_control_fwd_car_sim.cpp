
#include <ori/simcars/agents/full_control_fwd_car_sim.hpp>

#include <ori/simcars/agents/fwd_car_sim.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

FullControlFWDCarSim::FullControlFWDCarSim(FullControlFWDCar *full_control_fwd_car,
                                           temporal::Time start_time) :
    FullControlFWDCar(*full_control_fwd_car),

    sim_start_time(start_time),

    zero_cumil_lane_trans(0),
    zero_cumil_lane_trans_proxy(&zero_cumil_lane_trans),

    lane_encaps(&pos, map),
    prev_lane_encaps(&lane_encaps),
    lane_trans(&prev_lane_encaps, &lane_encaps, map),

    sim_cumil_lane_trans(&zero_cumil_lane_trans_proxy, &cumil_lane_trans, &sim_start_time),
    prev_cumil_lane_trans(&sim_cumil_lane_trans),

    cumil_lane_trans(&lane_trans, &prev_cumil_lane_trans)
{
}

simcars::causal::IEndogenousVariable<structures::stl::STLStackArray<uint64_t>>* FullControlFWDCarSim::get_lane_encaps()
{
    return &lane_encaps;
}

simcars::causal::IEndogenousVariable<FP_DATA_TYPE>* FullControlFWDCarSim::get_cumil_lane_trans_variable()
{
    return &cumil_lane_trans;
}

}
}
}
