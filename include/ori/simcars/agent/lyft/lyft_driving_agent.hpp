#pragma once

#include <ori/simcars/structures/stl/stl_dictionary.hpp>
#include <ori/simcars/agent/driving_agent_abstract.hpp>

#include <rapidjson/document.h>

namespace ori
{
namespace simcars
{
namespace agent
{
namespace lyft
{

class LyftDrivingAgent : public virtual ADrivingAgent
{
    std::string name;

    geometry::Vec min_spatial_limits, max_spatial_limits;
    temporal::Time min_temporal_limit, max_temporal_limit;

    IConstant<uint32_t> *id_constant;
    IConstant<bool> *ego_constant;
    IConstant<FP_DATA_TYPE> *bb_length_constant;
    IConstant<FP_DATA_TYPE> *bb_width_constant;
    IConstant<DrivingAgentClass> *driving_agent_class_constant;

    IVariable<geometry::Vec> *position_variable;
    IVariable<geometry::Vec> *linear_velocity_variable;
    IVariable<FP_DATA_TYPE> *aligned_linear_velocity_variable;
    IVariable<geometry::Vec> *linear_acceleration_variable;
    IVariable<FP_DATA_TYPE> *aligned_linear_acceleration_variable;
    IVariable<geometry::Vec> *external_linear_acceleration_variable;
    IVariable<FP_DATA_TYPE> *rotation_variable;
    IVariable<FP_DATA_TYPE> *steer_variable;
    IVariable<FP_DATA_TYPE> *angular_velocity_variable;
    IVariable<temporal::Duration> *ttc_variable;
    IVariable<temporal::Duration> *cumilative_collision_time_variable;

    IDrivingScene const *driving_scene;

protected:
    LyftDrivingAgent();

public:
    LyftDrivingAgent(IDrivingScene const *driving_scene, rapidjson::Value::ConstObject const &json_agent_data);

    ~LyftDrivingAgent();

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

    IDrivingAgent* driving_agent_deep_copy(IDrivingScene *driving_scene) const override;

    IDrivingScene const* get_driving_scene() const override;

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
}
