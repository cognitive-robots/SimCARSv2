
#include <ori/simcars/agents/ped_sim.hpp>

#include <ori/simcars/agents/point_mass_sim.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

PedSim::PedSim(Ped *ped, temporal::Time start_time) :
    PointMass(*ped),
    PointMassSim(ped, start_time),
    Ped(*ped),

    original_ped(ped)
{
}

}
}
}
