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
    std::shared_ptr<const IDrivingAgent> driving_agent;

    temporal::Time max_temporal_limit;

    structures::stl::STLDictionary<std::string, std::shared_ptr<const IValuelessVariable>> non_simulated_variable_dict;
    structures::stl::STLDictionary<std::string, std::shared_ptr<const ISimulatedValuelessVariable>> simulated_variable_dict;

    std::shared_ptr<const DrivingSimulationScene> driving_simulation_scene;
    temporal::Time latest_simulated_time;

    DrivingSimulationAgent();

public:
    DrivingSimulationAgent(std::shared_ptr<const IDrivingAgent> driving_agent,
                           std::shared_ptr<const ISimulationScene> simulation_scene,
                           temporal::Time simulation_start_time, bool allow_late_start = true);
    DrivingSimulationAgent(std::shared_ptr<const IDrivingAgent> driving_agent,
                           std::shared_ptr<const ISimulationScene> simulation_scene,
                           temporal::Time simulation_start_time, temporal::Time simulation_end_time,
                           bool allow_late_start = true);

    std::string get_name() const override;

    geometry::Vec get_min_spatial_limits() const override;
    geometry::Vec get_max_spatial_limits() const override;

    temporal::Time get_min_temporal_limit() const override;
    temporal::Time get_max_temporal_limit() const override;

    std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessConstant>>> get_constant_parameters() const override;
    std::shared_ptr<const IValuelessConstant> get_constant_parameter(const std::string& constant_name) const override;

    std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessVariable>>> get_variable_parameters() const override;
    std::shared_ptr<const IValuelessVariable> get_variable_parameter(const std::string& variable_name) const override;

    std::shared_ptr<structures::IArray<std::shared_ptr<const IValuelessEvent>>> get_events() const override;

    std::shared_ptr<IDrivingAgent> driving_agent_deep_copy() const override;

    std::shared_ptr<IDrivingAgentState> get_driving_agent_state(temporal::Time time, bool throw_on_out_of_range) const override;

    void propogate(temporal::Time time, std::shared_ptr<const IDrivingAgentState> state) const;
};

}
}
}
