
#include <ori/simcars/agents/fwd_car_sim.hpp>

#include <ori/simcars/agents/rect_rigid_body_sim.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

FWDCarSim::FWDCarSim(FWDCar const *fwd_car, temporal::Time start_time) :
    FWDCar(*fwd_car),
    RectRigidBody(*fwd_car),
    RectRigidBodySim(fwd_car, start_time)
{
}

}
}
}
