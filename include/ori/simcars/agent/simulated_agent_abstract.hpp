#pragma once

#include <ori/simcars/temporal/temporal_dictionary.hpp>
#include <ori/simcars/agent/agent_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class ASimulatedAgent : public AAgent
{
    const std::shared_ptr<const IAgent> base_agent;
    const temporal::Time sim_start_time;
    const temporal::Duration sim_time_step;
    const std::weak_ptr<const IScene> scene;

protected:
    mutable temporal::TemporalDictionary<std::shared_ptr<ASimulatedAgent::State>> timestamp_to_state_dict;

    temporal::Time get_sim_start_time() const;
    temporal::Duration get_sim_time_step() const;

    virtual std::shared_ptr<ASimulatedAgent::State> simulate(std::shared_ptr<const ASimulatedAgent::State> previous_state, temporal::Duration time_diff) const = 0;

public:
    ASimulatedAgent(std::shared_ptr<const IScene> scene, std::shared_ptr<const IAgent> base_agent, temporal::Time sim_start_time, temporal::Duration sim_time_step);
    ASimulatedAgent(std::shared_ptr<const IScene> scene, std::shared_ptr<const IAgent> base_agent, temporal::Duration sim_time_step);

    std::shared_ptr<const ASimulatedAgent::State> get_state(temporal::Time timestamp) const override;

    temporal::Time get_birth() const override;
    temporal::Time get_death() const override;

    uint32_t get_id() const override;
    bool is_ego() const override;
    bool is_ever_simulated() const override;
    bool is_simulated(temporal::Time timestamp) const override;
    FP_DATA_TYPE get_length() const override;
    FP_DATA_TYPE get_width() const override;
    ASimulatedAgent::Class get_class() const override;
    std::shared_ptr<const IScene> get_scene() const override;
};

}
}
}
