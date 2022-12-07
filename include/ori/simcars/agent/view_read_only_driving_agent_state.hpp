#pragma once

#include <ori/simcars/agent/driving_agent_interface.hpp>
#include <ori/simcars/agent/read_only_driving_agent_state_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class ViewReadOnlyDrivingAgentState : public virtual AReadOnlyDrivingAgentState
{
    IDrivingAgent const *agent;
    temporal::Time const time;

public:
    ViewReadOnlyDrivingAgentState(IDrivingAgent const *agent, temporal::Time time);

    std::string get_name() const override;

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
    temporal::Time get_time() const;
};

}
}
}
