#pragma once

#include <ori/simcars/structures/array_interface.hpp>
#include <ori/simcars/temporal/typedefs.hpp>
#include <ori/simcars/agents/fwd_car.hpp>
#include <ori/simcars/agents/control_fwd_car.hpp>
#include <ori/simcars/agents/plan_fwd_car.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class IFWDCarScene
{
public:
    virtual ~IFWDCarScene() = default;

    virtual temporal::Duration get_time_step_size() const = 0;
    virtual temporal::Time get_min_time() const = 0;
    virtual temporal::Time get_max_time() const = 0;
    virtual FWDCar* get_fwd_car(uint32_t id) const = 0;
    virtual ControlFWDCar* get_control_fwd_car(uint32_t id) const = 0;
    virtual PlanFWDCar* get_plan_fwd_car(uint32_t id) const = 0;
    virtual structures::IArray<FWDCar*> const* get_fwd_cars() const = 0;
    virtual structures::IArray<ControlFWDCar*> const* get_control_fwd_cars() const = 0;
    virtual structures::IArray<PlanFWDCar*> const* get_plan_fwd_cars() const = 0;

    virtual RectRigidBodyEnv* get_env() = 0;
};

}
}
}
