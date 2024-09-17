
#include <ori/simcars/agents/ped.hpp>

namespace ori
{
namespace simcars
{
namespace agents
{

Ped::Ped(uint64_t id_value, FP_DATA_TYPE mass_value) : PointMass(id_value, mass_value) {}

Ped::Ped(Ped const &ped) : PointMass(ped) {}

}
}
}
