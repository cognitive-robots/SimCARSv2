#pragma once

#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/agent/constant_interface.hpp>
#include <ori/simcars/agent/variable_interface.hpp>
#include <ori/simcars/agent/entity_interface.hpp>
#include <ori/simcars/agent/driving_enums.hpp>
#include <ori/simcars/agent/driving_agent_state_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class IDrivingAgent : public virtual IEntity
{
public:
    virtual IDrivingAgent* driving_agent_deep_copy() const = 0;


    virtual IConstant<uint32_t> const* get_id_constant() const = 0;
    virtual IConstant<bool> const* get_ego_constant() const = 0;
    virtual IConstant<FP_DATA_TYPE> const* get_bb_length_constant() const = 0;
    virtual IConstant<FP_DATA_TYPE> const* get_bb_width_constant() const = 0;
    virtual IConstant<DrivingAgentClass> const* get_driving_agent_class_constant() const = 0;

    virtual IVariable<geometry::Vec> const* get_position_variable() const = 0;
    virtual IVariable<geometry::Vec> const* get_linear_velocity_variable() const = 0;
    virtual IVariable<FP_DATA_TYPE> const* get_aligned_linear_velocity_variable() const = 0;
    virtual IVariable<geometry::Vec> const* get_linear_acceleration_variable() const = 0;
    virtual IVariable<FP_DATA_TYPE> const* get_aligned_linear_acceleration_variable() const = 0;
    virtual IVariable<geometry::Vec> const* get_external_linear_acceleration_variable() const = 0;
    virtual IVariable<FP_DATA_TYPE> const* get_rotation_variable() const = 0;
    virtual IVariable<FP_DATA_TYPE> const* get_steer_variable() const = 0;
    virtual IVariable<FP_DATA_TYPE> const* get_angular_velocity_variable() const = 0;

    virtual IReadOnlyDrivingAgentState const* get_driving_agent_state(temporal::Time time) const = 0;


    virtual IConstant<uint32_t>* get_mutable_id_constant() = 0;
    virtual IConstant<bool>* get_mutable_ego_constant() = 0;
    virtual IConstant<FP_DATA_TYPE>* get_mutable_bb_length_constant() = 0;
    virtual IConstant<FP_DATA_TYPE>* get_mutable_bb_width_constant() = 0;
    virtual IConstant<DrivingAgentClass>* get_mutable_driving_agent_class_constant() = 0;

    virtual IVariable<geometry::Vec>* get_mutable_position_variable() = 0;
    virtual IVariable<geometry::Vec>* get_mutable_linear_velocity_variable() = 0;
    virtual IVariable<FP_DATA_TYPE>* get_mutable_aligned_linear_velocity_variable() = 0;
    virtual IVariable<geometry::Vec>* get_mutable_linear_acceleration_variable() = 0;
    virtual IVariable<FP_DATA_TYPE>* get_mutable_aligned_linear_acceleration_variable() = 0;
    virtual IVariable<geometry::Vec>* get_mutable_external_linear_acceleration_variable() = 0;
    virtual IVariable<FP_DATA_TYPE>* get_mutable_rotation_variable() = 0;
    virtual IVariable<FP_DATA_TYPE>* get_mutable_steer_variable() = 0;
    virtual IVariable<FP_DATA_TYPE>* get_mutable_angular_velocity_variable() = 0;

    virtual IDrivingAgentState* get_mutable_driving_agent_state(temporal::Time time) = 0;
};

}
}
}
