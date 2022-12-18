#pragma once

#include <ori/simcars/temporal/precedence_temporal_dictionary.hpp>
#include <ori/simcars/agent/declarations.hpp>
#include <ori/simcars/agent/basic_driving_agent_state.hpp>
#include <ori/simcars/agent/simulated_valueless_variable_interface.hpp>
#include <ori/simcars/agent/driving_simulation_scene_interface.hpp>
#include <ori/simcars/agent/driving_simulation_agent_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class DrivingSimulationAgent : public virtual ADrivingSimulationAgent
{
    IDrivingAgent *driving_agent;

    mutable temporal::Time simulation_start_time;
    temporal::Time simulation_end_time;
    temporal::Time latest_simulated_time;

    structures::stl::STLDictionary<std::string, ISimulatedValuelessVariable*> simulated_variable_dict;

    IDrivingSimulationScene const *driving_simulation_scene;

protected:
    DrivingSimulationAgent();

public:
    DrivingSimulationAgent(IDrivingAgent *driving_agent,
                           ISimulationScene *simulation_scene,
                           temporal::Time simulation_start_time,
                           bool start_simulated,
                           bool allow_late_start = true);
    DrivingSimulationAgent(IDrivingAgent *driving_agent,
                           ISimulationScene *simulation_scene,
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

    bool is_state_available(temporal::Time) const override;

    structures::IArray<IValuelessConstant const*>* get_constant_parameters() const override;
    IValuelessConstant const* get_constant_parameter(std::string const &constant_name) const override;

    structures::IArray<IValuelessVariable const*>* get_variable_parameters() const override;
    IValuelessVariable const* get_variable_parameter(std::string const &variable_name) const override;

    structures::IArray<IValuelessEvent const*>* get_events() const override;

    IDrivingSimulationAgent* driving_simulation_agent_deep_copy(IDrivingSimulationScene *driving_simulation_scene) const override;

    IDrivingSimulationScene const* get_driving_simulation_scene() const override;

    void begin_simulation(temporal::Time simulation_start_time) const override;


    structures::IArray<IValuelessConstant*>* get_mutable_constant_parameters() override;
    IValuelessConstant* get_mutable_constant_parameter(std::string const &constant_name) override;

    structures::IArray<IValuelessVariable*>* get_mutable_variable_parameters() override;
    IValuelessVariable* get_mutable_variable_parameter(std::string const &variable_name) override;

    structures::IArray<IValuelessEvent*>* get_mutable_events() override;
};

}
}
}
