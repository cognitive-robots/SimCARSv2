#pragma once

#include <ori/simcars/temporal/typedefs.hpp>
#include <ori/simcars/agent/scene_state_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class ISimulator
{
public:
    virtual ~ISimulator() = default;

    virtual void simulate(agent::IReadOnlySceneState const *current_state, agent::ISceneState *next_state, temporal::Duration time_step) const = 0;
};

}
}
}
