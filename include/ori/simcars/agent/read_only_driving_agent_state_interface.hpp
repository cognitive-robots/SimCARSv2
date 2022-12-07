#pragma once

#include <ori/simcars/geometry/typedefs.hpp>
#include <ori/simcars/agent/constant_interface.hpp>
#include <ori/simcars/agent/read_only_entity_state_interface.hpp>
#include <ori/simcars/agent/driving_enums.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class IReadOnlyDrivingAgentState : public virtual IReadOnlyEntityState
{
public:
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
    virtual IConstant<temporal::Duration> const* get_ttc_variable() const = 0;
    virtual IConstant<temporal::Duration> const* get_cumilative_collision_time_variable() const = 0;
};

}
}
}
