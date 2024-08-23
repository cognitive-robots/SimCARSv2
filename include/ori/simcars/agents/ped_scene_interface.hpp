#pragma once

#include <ori/simcars/structures/array_interface.hpp>
#include <ori/simcars/temporal/typedefs.hpp>
#include <ori/simcars/agents/ped.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class IPedScene
{
public:
    virtual ~IPedScene() = default;

    virtual temporal::Duration get_time_step_size() const = 0;
    virtual temporal::Time get_min_time() const = 0;
    virtual temporal::Time get_max_time() const = 0;
    virtual Ped* get_ped(uint32_t id) const = 0;
    virtual structures::IArray<Ped*> const* get_peds() const = 0;

    virtual PointMassEnv* get_env() = 0;
};

}
}
}
