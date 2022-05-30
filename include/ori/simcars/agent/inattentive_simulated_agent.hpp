#pragma once

#include <ori/simcars/geometry/trig_buff.hpp>
#include <ori/simcars/agent/simulated_agent_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class InattentiveSimulatedAgent : public ASimulatedAgent
{
protected:
    std::shared_ptr<const geometry::TrigBuff> trig_buff;

    std::shared_ptr<ASimulatedAgent::State> simulate(std::shared_ptr<const ASimulatedAgent::State> previous_state, temporal::Duration time_diff) const override;

    std::shared_ptr<ASimulatedAgent::State> simulate_physics(std::shared_ptr<const ASimulatedAgent::State> previous_state, temporal::Duration time_diff) const;

public:
    InattentiveSimulatedAgent(std::shared_ptr<const IScene> scene, std::shared_ptr<const IAgent> base_agent, temporal::Time sim_start_time, temporal::Duration sim_time_step);
};

}
}
}
