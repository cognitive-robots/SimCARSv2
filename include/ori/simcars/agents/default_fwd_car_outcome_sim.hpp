#pragma once

#include <ori/simcars/agents/fwd_car_outcome_sim_interface.hpp>
#include <ori/simcars/agents/rect_rigid_body_env.hpp>
#include <ori/simcars/agents/full_control_fwd_car.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class DefaultFWDCarOutcomeSim : public virtual IFWDCarOutcomeSim
{
    FullControlFWDCar *control_fwd_car;
    RectRigidBodyEnv *rigid_body_env;

public:
    DefaultFWDCarOutcomeSim(FullControlFWDCar *control_fwd_car,
                            RectRigidBodyEnv *rigid_body_env);

    FWDCarOutcome sim_outcome(FWDCarAction const *action,
                              FWDCarSimParameters const *parameters) const override;
};

}
}
}
