#pragma once

#include <ori/simcars/geometry/defines.hpp>
#include <ori/simcars/agent/entity_state_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class IAgencyCalculator
{
public:
    virtual ~IAgencyCalculator() = default;

    virtual bool calculate_state_agency(agent::IReadOnlyEntityState const *state) const = 0;
};

}
}
}
