#pragma once

#include <ori/simcars/structures/array_interface.hpp>
#include <ori/simcars/temporal/typedefs.hpp>
#include <ori/simcars/agents/point_mass.hpp>

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
    virtual PointMass* get_point_mass(uint32_t id) const = 0;
    virtual structures::IArray<PointMass*> const* get_point_masses() const = 0;

    //virtual RectRigidBodyEnv* get_env() = 0;
};

}
}
}
