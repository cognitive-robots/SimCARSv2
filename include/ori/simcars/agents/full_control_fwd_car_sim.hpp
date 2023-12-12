#pragma once

#include <ori/simcars/agents/full_control_fwd_car.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class FullControlFWDCarSim : public virtual FullControlFWDCar
{
public:
    FullControlFWDCarSim(FullControlFWDCar const *full_control_fwd_car,
                         FWDCar *fwd_car, temporal::Time start_time);
};

}
}
}
