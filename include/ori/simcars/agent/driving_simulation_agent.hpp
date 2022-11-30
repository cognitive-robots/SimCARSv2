#pragma once

#include <ori/simcars/temporal/precedence_temporal_dictionary.hpp>
#include <ori/simcars/agent/declarations.hpp>
#include <ori/simcars/agent/driving_agent_abstract.hpp>
#include <ori/simcars/agent/basic_driving_agent_state.hpp>
#include <ori/simcars/agent/simulated_valueless_variable_interface.hpp>
#include <ori/simcars/agent/driving_simulation_scene.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class DrivingSimulationAgent : public virtual ADrivingAgent
{
    IDrivingAgent const *driving_agent;

    mutable temporal::Time simulation_start_time;
    temporal::Time simulation_end_time;

    structures::stl::STLDictionary<std::string, IValuelessVariable const*> non_simulated_variable_dict;
    structures::stl::STLDictionary<std::string, ISimulatedValuelessVariable const*> simulated_variable_dict;

    DrivingSimulationScene const *driving_simulation_scene;
    temporal::Time latest_simulated_time;

    DrivingSimulationAgent();

public:
    DrivingSimulationAgent(IDrivingAgent const *driving_agent,
                           ISimulationScene const *simulation_scene,
                           temporal::Time simulation_start_time,
                           bool start_simulated,
                           bool allow_late_start = true);
    DrivingSimulationAgent(IDrivingAgent const *driving_agent,
                           ISimulationScene const *simulation_scene,
                           temporal::Time simulation_start_time,
                           temporal::Time simulation_end_time,
                           bool start_simulated,
                           bool allow_late_start = true);

    ~DrivingSimulationAgent();

    std::string get_name() const override;

    geometry::Vec get_min_spatial_limits() const override;
    geometry::Vec get_max_spatial_limits() const override;

    temporal::Time get_min_temporal_limit() const override;
    temporal::Time get_max_temporal_limit() const override;

    structures::IArray<IValuelessConstant const*>* get_constant_parameters() const override;
    IValuelessConstant const* get_constant_parameter(std::string const &constant_name) const override;

    structures::IArray<IValuelessVariable const*>* get_variable_parameters() const override;
    IValuelessVariable const* get_variable_parameter(std::string const &variable_name) const override;

    structures::IArray<IValuelessEvent const*>* get_events() const override;

    IDrivingAgent* driving_agent_deep_copy() const override;

    IDrivingAgentState* get_driving_agent_state(temporal::Time time, bool throw_on_out_of_range) const override;

    void propogate(temporal::Time time, IDrivingAgentState const *state) const;

    void begin_simulation(temporal::Time simulation_start_time) const;
};

}
}
}
