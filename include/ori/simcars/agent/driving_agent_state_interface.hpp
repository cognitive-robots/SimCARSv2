#pragma once

#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/agent/constant_interface.hpp>
#include <ori/simcars/agent/state_interface.hpp>
#include <ori/simcars/agent/driving_enums.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class IDrivingAgentState : public virtual IState
{
public:
    virtual std::string get_driving_agent_name() const = 0;

    virtual IConstant<uint32_t> const* get_id_constant() const = 0;
    virtual IConstant<bool> const* get_ego_constant() const = 0;
    virtual IConstant<FP_DATA_TYPE> const* get_bb_length_constant() const = 0;
    virtual IConstant<FP_DATA_TYPE> const* get_bb_width_constant() const = 0;
    virtual IConstant<DrivingAgentClass> const* get_driving_agent_class_constant() const = 0;
    virtual IConstant<geometry::Vec> const* get_position_variable() const = 0;
    virtual IConstant<geometry::Vec> const* get_linear_velocity_variable() const = 0;
    virtual IConstant<FP_DATA_TYPE> const* get_aligned_linear_velocity_variable() const = 0;
    virtual IConstant<geometry::Vec> const* get_linear_acceleration_variable() const = 0;
    virtual IConstant<FP_DATA_TYPE> const* get_aligned_linear_acceleration_variable() const = 0;
    virtual IConstant<geometry::Vec> const* get_external_linear_acceleration_variable() const = 0;
    virtual IConstant<FP_DATA_TYPE> const* get_rotation_variable() const = 0;
    virtual IConstant<FP_DATA_TYPE> const* get_steer_variable() const = 0;
    virtual IConstant<FP_DATA_TYPE> const* get_angular_velocity_variable() const = 0;

    virtual void set_driving_agent_name(std::string const &driving_agent_name) = 0;

    virtual void set_id_constant(IConstant<uint32_t> const *id_constant) = 0;
    virtual void set_ego_constant(IConstant<bool> const *ego_constant) = 0;
    virtual void set_bb_length_constant(IConstant<FP_DATA_TYPE> const *bb_length_constant) = 0;
    virtual void set_bb_width_constant(IConstant<FP_DATA_TYPE> const *bb_width_constant) = 0;
    virtual void set_driving_agent_class_constant(IConstant<DrivingAgentClass> const *driving_agent_class_constant) = 0;
    virtual void set_position_variable(IConstant<geometry::Vec> const *position_variable) = 0;
    virtual void set_linear_velocity_variable(IConstant<geometry::Vec> const *linear_velocity_variable) = 0;
    virtual void set_aligned_linear_velocity_variable(IConstant<FP_DATA_TYPE> const *aligned_linear_velocity_variable) = 0;
    virtual void set_linear_acceleration_variable(IConstant<geometry::Vec> const *linear_acceleration_variable) = 0;
    virtual void set_aligned_linear_acceleration_variable(IConstant<FP_DATA_TYPE> const *aligned_linear_acceleration_variable) = 0;
    virtual void set_external_linear_acceleration_variable(IConstant<geometry::Vec> const *external_linear_acceleration_variable) = 0;
    virtual void set_rotation_variable(IConstant<FP_DATA_TYPE> const *rotation_variable) = 0;
    virtual void set_steer_variable(IConstant<FP_DATA_TYPE> const *steer_variable) = 0;
    virtual void set_angular_velocity_variable(IConstant<FP_DATA_TYPE> const *angular_velocity_variable) = 0;
};

}
}
}
