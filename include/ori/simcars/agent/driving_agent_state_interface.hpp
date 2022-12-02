#pragma once

#include <ori/simcars/agent/entity_state_interface.hpp>
#include <ori/simcars/agent/read_only_driving_agent_state_interface.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class IDrivingAgentState : public virtual IReadOnlyDrivingAgentState, public virtual IEntityState
{
public:
    virtual void set_id_constant(IConstant<uint32_t> *id_constant) = 0;
    virtual void set_ego_constant(IConstant<bool> *ego_constant) = 0;
    virtual void set_bb_length_constant(IConstant<FP_DATA_TYPE> *bb_length_constant) = 0;
    virtual void set_bb_width_constant(IConstant<FP_DATA_TYPE> *bb_width_constant) = 0;
    virtual void set_driving_agent_class_constant(IConstant<DrivingAgentClass> *driving_agent_class_constant) = 0;
    virtual void set_position_variable(IConstant<geometry::Vec> *position_variable) = 0;
    virtual void set_linear_velocity_variable(IConstant<geometry::Vec> *linear_velocity_variable) = 0;
    virtual void set_aligned_linear_velocity_variable(IConstant<FP_DATA_TYPE> *aligned_linear_velocity_variable) = 0;
    virtual void set_linear_acceleration_variable(IConstant<geometry::Vec> *linear_acceleration_variable) = 0;
    virtual void set_aligned_linear_acceleration_variable(IConstant<FP_DATA_TYPE> *aligned_linear_acceleration_variable) = 0;
    virtual void set_external_linear_acceleration_variable(IConstant<geometry::Vec> *external_linear_acceleration_variable) = 0;
    virtual void set_rotation_variable(IConstant<FP_DATA_TYPE> *rotation_variable) = 0;
    virtual void set_steer_variable(IConstant<FP_DATA_TYPE> *steer_variable) = 0;
    virtual void set_angular_velocity_variable(IConstant<FP_DATA_TYPE> *angular_velocity_variable) = 0;
};

}
}
}
