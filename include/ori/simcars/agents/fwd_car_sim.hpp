#pragma once

#include <ori/simcars/agents/rect_rigid_body_sim.hpp>
#include <ori/simcars/agents/fwd_car.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class FWDCarSim : public FWDCar, public RectRigidBodySim
{
public:
    FWDCarSim(FWDCar *fwd_car, temporal::Time start_time);
};

}
}
}
