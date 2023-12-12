
#include <ori/simcars/agents/full_control_fwd_car_sim.hpp>

#include <ori/simcars/agents/fwd_car_sim.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

FullControlFWDCarSim::FullControlFWDCarSim(FullControlFWDCar const *full_control_fwd_car,
                                           FWDCar *fwd_car, temporal::Time start_time) :
    FullControlFWDCar(*full_control_fwd_car), MotorTorqueControlFWDCar(*full_control_fwd_car),
    SteerControlFWDCar(*full_control_fwd_car)
{
    set_fwd_car(fwd_car);
}

}
}
}
