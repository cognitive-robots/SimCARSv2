#pragma once

#include <ori/simcars/structures/array_interface.hpp>
#include <ori/simcars/temporal/typedefs.hpp>
#include <ori/simcars/agents/fwd_car.hpp>

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
    virtual structures::IArray<FWDCar*> const* get_fwd_cars() const = 0;
};

}
}
}
