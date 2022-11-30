#pragma once

#include <ori/simcars/structures/stl/stl_dictionary.hpp>
#include <ori/simcars/agent/driving_agent_state_abstract.hpp>

namespace ori
{
namespace simcars
{
namespace agent
{

class BasicDrivingAgentState : public virtual ADrivingAgentState
{
    std::string driving_agent_name;
    bool delete_dicts;

protected:
    structures::stl::STLDictionary<std::string, IValuelessConstant const*> parameter_dict;

public:
    BasicDrivingAgentState(std::string const &driving_agent_name, bool delete_dicts = true);
    BasicDrivingAgentState(IDrivingAgentState const *driving_agent_state, bool copy_parameters = true);

    ~BasicDrivingAgentState();

    structures::IArray<IValuelessConstant const*>* get_parameter_values() const override;
    IValuelessConstant const* get_parameter_value(std::string const &parameter_name) const override;

    std::string get_driving_agent_name() const override;

    void set_parameter_values(structures::IArray<IValuelessConstant const*> const *parameter_values) override;
    void set_parameter_value(IValuelessConstant const *parameter_value) override;

    void set_driving_agent_name(std::string const &driving_agent_name) override;

    void set_id_constant(IConstant<uint32_t> const *id_constant) override;
    void set_ego_constant(IConstant<bool> const *ego_constant) override;
    void set_bb_length_constant(IConstant<FP_DATA_TYPE> const *bb_length_constant) override;
    void set_bb_width_constant(IConstant<FP_DATA_TYPE> const *bb_width_constant) override;
    void set_driving_agent_class_constant(IConstant<DrivingAgentClass> const *driving_agent_class_constant) override;
    void set_position_variable(IConstant<geometry::Vec> const *position_variable) override;
    void set_linear_velocity_variable(IConstant<geometry::Vec> const *linear_velocity_variable) override;
    void set_aligned_linear_velocity_variable(IConstant<FP_DATA_TYPE> const *aligned_linear_velocity_variable) override;
    void set_linear_acceleration_variable(IConstant<geometry::Vec> const *linear_acceleration_variable) override;
    void set_aligned_linear_acceleration_variable(IConstant<FP_DATA_TYPE> const *aligned_linear_acceleration_variable) override;
    void set_external_linear_acceleration_variable(IConstant<geometry::Vec> const *external_linear_acceleration_variable) override;
    void set_rotation_variable(IConstant<FP_DATA_TYPE> const *rotation_variable) override;
    void set_steer_variable(IConstant<FP_DATA_TYPE> const *steer_variable) override;
    void set_angular_velocity_variable(IConstant<FP_DATA_TYPE> const *angular_velocity_variable) override;
};

}
}
}
