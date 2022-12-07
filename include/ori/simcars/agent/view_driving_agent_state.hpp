#pragma once

#include <ori/simcars/agent/driving_agent_interface.hpp>
#include <ori/simcars/agent/driving_agent_state_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class ViewDrivingAgentState : public virtual ADrivingAgentState
{
    IDrivingAgent *agent;
    temporal::Time time;

public:
    ViewDrivingAgentState(IDrivingAgent *agent, temporal::Time time);

    std::string get_name() const override;
    temporal::Time get_time() const override;

    bool is_populated() const override;

    structures::IArray<IValuelessConstant const*>* get_parameter_values() const override;
    IValuelessConstant const* get_parameter_value(std::string const &parameter_name) const override;

    IConstant<uint32_t> const* get_id_constant() const override;
    IConstant<bool> const* get_ego_constant() const override;
    IConstant<FP_DATA_TYPE> const* get_bb_length_constant() const override;
    IConstant<FP_DATA_TYPE> const* get_bb_width_constant() const override;
    IConstant<DrivingAgentClass> const* get_driving_agent_class_constant() const override;
    IConstant<geometry::Vec> const* get_position_variable() const override;
    IConstant<geometry::Vec> const* get_linear_velocity_variable() const override;
    IConstant<FP_DATA_TYPE> const* get_aligned_linear_velocity_variable() const override;
    IConstant<geometry::Vec> const* get_linear_acceleration_variable() const override;
    IConstant<FP_DATA_TYPE> const* get_aligned_linear_acceleration_variable() const override;
    IConstant<geometry::Vec> const* get_external_linear_acceleration_variable() const override;
    IConstant<FP_DATA_TYPE> const* get_rotation_variable() const override;
    IConstant<FP_DATA_TYPE> const* get_steer_variable() const override;
    IConstant<FP_DATA_TYPE> const* get_angular_velocity_variable() const override;
    IConstant<temporal::Duration> const* get_ttc_variable() const override;
    IConstant<temporal::Duration> const* get_cumilative_collision_time_variable() const override;

    IDrivingAgent const* get_agent() const;


    structures::IArray<IValuelessConstant*>* get_mutable_parameter_values() override;
    IValuelessConstant* get_mutable_parameter_value(std::string const &parameter_name) override;

    void set_id_constant(IConstant<uint32_t> *id_constant) override;
    void set_ego_constant(IConstant<bool> *ego_constant) override;
    void set_bb_length_constant(IConstant<FP_DATA_TYPE> *bb_length_constant) override;
    void set_bb_width_constant(IConstant<FP_DATA_TYPE> *bb_width_constant) override;
    void set_driving_agent_class_constant(IConstant<DrivingAgentClass> *driving_agent_class_constant) override;
    void set_position_variable(IConstant<geometry::Vec> *position_variable) override;
    void set_linear_velocity_variable(IConstant<geometry::Vec> *linear_velocity_variable) override;
    void set_aligned_linear_velocity_variable(IConstant<FP_DATA_TYPE> *aligned_linear_velocity_variable) override;
    void set_linear_acceleration_variable(IConstant<geometry::Vec> *linear_acceleration_variable) override;
    void set_aligned_linear_acceleration_variable(IConstant<FP_DATA_TYPE> *aligned_linear_acceleration_variable) override;
    void set_external_linear_acceleration_variable(IConstant<geometry::Vec> *external_linear_acceleration_variable) override;
    void set_rotation_variable(IConstant<FP_DATA_TYPE> *rotation_variable) override;
    void set_steer_variable(IConstant<FP_DATA_TYPE> *steer_variable) override;
    void set_angular_velocity_variable(IConstant<FP_DATA_TYPE> *angular_velocity_variable) override;
    void set_ttc_variable(IConstant<temporal::Duration> *ttc_variable) override;
    void set_cumilative_collision_time_variable(IConstant<temporal::Duration> *cumilative_collision_time_variable) override;

    void set_agent(IDrivingAgent *agent);
    void set_time(temporal::Time time);
};

}
}
}
