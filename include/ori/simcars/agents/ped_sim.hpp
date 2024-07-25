#pragma once

#include <ori/simcars/agents/point_mass_sim.hpp>
#include <ori/simcars/agents/ped.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

class PedSim : public Ped, public PointMassSim
{
    Ped const* const original_ped;

public:
    PedSim(Ped *ped, temporal::Time start_time);
};

}
}
}
