#pragma once

#include <ori/simcars/temporal/precedence_temporal_dictionary.hpp>
#include <ori/simcars/agent/declarations.hpp>
#include <ori/simcars/agent/basic_driving_agent_state.hpp>
#include <ori/simcars/agent/simulated_variable_interface.hpp>
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

    ISimulatedVariable<geometry::Vec> *position_variable;
    ISimulatedVariable<geometry::Vec> *linear_velocity_variable;
    ISimulatedVariable<FP_DATA_TYPE> *aligned_linear_velocity_variable;
    ISimulatedVariable<geometry::Vec> *linear_acceleration_variable;
    ISimulatedVariable<FP_DATA_TYPE> *aligned_linear_acceleration_variable;
    ISimulatedVariable<geometry::Vec> *external_linear_acceleration_variable;
    ISimulatedVariable<FP_DATA_TYPE> *rotation_variable;
    ISimulatedVariable<FP_DATA_TYPE> *steer_variable;
    ISimulatedVariable<FP_DATA_TYPE> *angular_velocity_variable;
    ISimulatedVariable<temporal::Duration> *ttc_variable;
    ISimulatedVariable<temporal::Duration> *cumilative_collision_time_variable;

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

    IConstant<uint32_t> const* get_id_constant() const override;
    IConstant<bool> const* get_ego_constant() const override;
    IConstant<FP_DATA_TYPE> const* get_bb_length_constant() const override;
    IConstant<FP_DATA_TYPE> const* get_bb_width_constant() const override;
    IConstant<DrivingAgentClass> const* get_driving_agent_class_constant() const override;

    IVariable<geometry::Vec> const* get_position_variable() const override;
    IVariable<geometry::Vec> const* get_linear_velocity_variable() const override;
    IVariable<FP_DATA_TYPE> const* get_aligned_linear_velocity_variable() const override;
    IVariable<geometry::Vec> const* get_linear_acceleration_variable() const override;
    IVariable<FP_DATA_TYPE> const* get_aligned_linear_acceleration_variable() const override;
    IVariable<geometry::Vec> const* get_external_linear_acceleration_variable() const override;
    IVariable<FP_DATA_TYPE> const* get_rotation_variable() const override;
    IVariable<FP_DATA_TYPE> const* get_steer_variable() const override;
    IVariable<FP_DATA_TYPE> const* get_angular_velocity_variable() const override;
    IVariable<temporal::Duration> const* get_ttc_variable() const override;
    IVariable<temporal::Duration> const* get_cumilative_collision_time_variable() const override;

    IDrivingSimulationAgent* driving_simulation_agent_deep_copy(IDrivingSimulationScene *driving_simulation_scene) const override;

    IDrivingSimulationScene const* get_driving_simulation_scene() const override;

    void begin_simulation(temporal::Time simulation_start_time) const override;


    structures::IArray<IValuelessConstant*>* get_mutable_constant_parameters() override;
    IValuelessConstant* get_mutable_constant_parameter(std::string const &constant_name) override;

    structures::IArray<IValuelessVariable*>* get_mutable_variable_parameters() override;
    IValuelessVariable* get_mutable_variable_parameter(std::string const &variable_name) override;

    structures::IArray<IValuelessEvent*>* get_mutable_events() override;

    IConstant<uint32_t>* get_mutable_id_constant() override;
    IConstant<bool>* get_mutable_ego_constant() override;
    IConstant<FP_DATA_TYPE>* get_mutable_bb_length_constant() override;
    IConstant<FP_DATA_TYPE>* get_mutable_bb_width_constant() override;
    IConstant<DrivingAgentClass>* get_mutable_driving_agent_class_constant() override;

    IVariable<geometry::Vec>* get_mutable_position_variable() override;
    IVariable<geometry::Vec>* get_mutable_linear_velocity_variable() override;
    IVariable<FP_DATA_TYPE>* get_mutable_aligned_linear_velocity_variable() override;
    IVariable<geometry::Vec>* get_mutable_linear_acceleration_variable() override;
    IVariable<FP_DATA_TYPE>* get_mutable_aligned_linear_acceleration_variable() override;
    IVariable<geometry::Vec>* get_mutable_external_linear_acceleration_variable() override;
    IVariable<FP_DATA_TYPE>* get_mutable_rotation_variable() override;
    IVariable<FP_DATA_TYPE>* get_mutable_steer_variable() override;
    IVariable<FP_DATA_TYPE>* get_mutable_angular_velocity_variable() override;
    IVariable<temporal::Duration>* get_mutable_ttc_variable() override;
    IVariable<temporal::Duration>* get_mutable_cumilative_collision_time_variable() override;
};

}
}
}
