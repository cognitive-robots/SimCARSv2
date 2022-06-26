#pragma once

#include <ori/simcars/agent/scene_interface.hpp>
#include <ori/simcars/agent/simulator_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class DrivingAgentSimulator : public virtual ISimulator
{
    std::shared_ptr<const IScene> scene;

public:
    DrivingAgentSimulator(std::shared_ptr<const IScene> scene);

    std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessEvent>>> simulate(temporal::Time current_time, temporal::Duration time_step) const override;
};

}
}
}
