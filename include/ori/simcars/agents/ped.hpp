#pragma once

#include <ori/simcars/agents/declarations.hpp>
#include <ori/simcars/agents/point_mass.hpp>
#include <ori/simcars/agents/control_ped.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class Ped : public virtual PointMass
{
public:
    Ped(uint64_t id_value, FP_DATA_TYPE mass_value);
    Ped(Ped const &ped);

    friend void ControlPed::set_ped(Ped *ped);
};

}
}
}
