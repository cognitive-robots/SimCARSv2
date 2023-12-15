#pragma once

#include <ori/simcars/agents/rect_rigid_body_sim.hpp>
#include <ori/simcars/agents/fwd_car.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class FWDCarSim : public virtual FWDCar, public virtual RectRigidBodySim
{
public:
    FWDCarSim(FWDCar const *fwd_car, temporal::Time start_time);
};

}
}
}
