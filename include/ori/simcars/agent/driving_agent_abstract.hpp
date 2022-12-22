#pragma once

#include <ori/simcars/agent/driving_agent_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class ADrivingAgent : public virtual IDrivingAgent
{
public:
    IEntity* entity_deep_copy(IScene *scene) const override;

    IScene const* get_scene() const override;

    temporal::Time get_last_event_time() const override;

    bool is_state_available(temporal::Time) const override;

    IReadOnlyEntityState const* get_state(temporal::Time time) const override;

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

    IReadOnlyDrivingAgentState const* get_driving_agent_state(temporal::Time time) const override;


    IEntityState* get_mutable_state(temporal::Time time) override;

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

    IDrivingAgentState* get_mutable_driving_agent_state(temporal::Time time) override;
};

}
}
}
